import CoreBluetooth
import SwiftUI

final class SensorBLEManager: NSObject, ObservableObject {
    private static let serviceUUID = CBUUID(string: "b22d7f14-9361-4309-932e-ffbdefed97fe")
    private static let co2UUID = CBUUID(string: "c7d0c8a8-db04-4199-869d-5f80091f2036")
    private static let temperatureUUID = CBUUID(string: "c7d0c8a8-db04-4199-869d-5f80091f2037")
    private static let scanTimeoutNanoseconds: UInt64 = 12 * 1_000_000_000

    @Published private(set) var phase: ConnectionPhase = .idle
    @Published private(set) var co2PPM: UInt16?
    @Published private(set) var temperatureC: Int16?
    @Published private(set) var lastUpdate: Date?

    private var centralManager: CBCentralManager!
    private var sensorPeripheral: CBPeripheral?
    private var co2Characteristic: CBCharacteristic?
    private var temperatureCharacteristic: CBCharacteristic?
    private var pendingNotificationUUIDs = Set<CBUUID>()
    private var scanTimeoutWorkItem: DispatchWorkItem?
    private var shouldConnectWhenPoweredOn = false
    private var isDisconnectingByUser = false
    private var pendingFailureMessage: String?

    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: .main)
    }

    func start() {
        shouldConnectWhenPoweredOn = true
        isDisconnectingByUser = false

        switch centralManager.state {
        case .poweredOn:
            scanForSensor()
        case .poweredOff:
            phase = .bluetoothUnavailable("Bluetooth is off.")
        case .unauthorized:
            phase = .bluetoothUnavailable("Bluetooth permission is not granted.")
        case .unsupported:
            phase = .bluetoothUnavailable("This device does not support Bluetooth LE.")
        case .resetting:
            phase = .waitingForBluetooth
        case .unknown:
            phase = .waitingForBluetooth
        @unknown default:
            phase = .bluetoothUnavailable("Bluetooth is unavailable.")
        }
    }

    func performPrimaryAction() {
        switch phase {
        case .connected:
            disconnect()
        case .idle, .failed, .disconnected, .bluetoothUnavailable:
            start()
        case .waitingForBluetooth, .scanning, .connecting, .discovering, .subscribing:
            break
        }
    }

    private func scanForSensor() {
        cleanupConnectionState()
        co2PPM = nil
        temperatureC = nil
        lastUpdate = nil
        phase = .scanning

        centralManager.scanForPeripherals(
            withServices: [Self.serviceUUID],
            options: [CBCentralManagerScanOptionAllowDuplicatesKey: false]
        )

        cancelScanTimeout()

        let timeoutWorkItem = DispatchWorkItem { [weak self] in
            self?.handleScanTimeout()
        }
        scanTimeoutWorkItem = timeoutWorkItem
        DispatchQueue.main.asyncAfter(
            deadline: .now() + .nanoseconds(Int(Self.scanTimeoutNanoseconds)),
            execute: timeoutWorkItem
        )
    }

    private func connect(to peripheral: CBPeripheral, advertisedName: String?) {
        cancelScanTimeout()
        centralManager.stopScan()

        sensorPeripheral = peripheral
        sensorPeripheral?.delegate = self
        phase = .connecting(displayName(for: peripheral, advertisedName: advertisedName))
        centralManager.connect(peripheral, options: nil)
    }

    private func disconnect() {
        shouldConnectWhenPoweredOn = false
        isDisconnectingByUser = true
        cancelScanTimeout()
        centralManager.stopScan()

        if let sensorPeripheral {
            centralManager.cancelPeripheralConnection(sensorPeripheral)
        } else {
            cleanupConnectionState()
            phase = .disconnected("Disconnected.")
        }
    }

    private func handleScanTimeout() {
        guard phase == .scanning else {
            return
        }

        centralManager.stopScan()
        phase = .failed("No RustCO2Sensor advertising the CO2 service was found.")
    }

    private func discoverMeasurementCharacteristics(on peripheral: CBPeripheral) {
        guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else {
            fail("The connected peripheral does not expose the CO2 service.")
            return
        }

        phase = .discovering(displayName(for: peripheral))
        peripheral.discoverCharacteristics([Self.co2UUID, Self.temperatureUUID], for: service)
    }

    private func subscribeToMeasurements(on peripheral: CBPeripheral, service: CBService) {
        guard let characteristics = service.characteristics else {
            fail("The CO2 service did not return characteristics.")
            return
        }

        guard let co2Characteristic = characteristics.first(where: { $0.uuid == Self.co2UUID }) else {
            fail("The CO2 concentration characteristic is missing.")
            return
        }

        guard let temperatureCharacteristic = characteristics.first(where: { $0.uuid == Self.temperatureUUID }) else {
            fail("The temperature characteristic is missing.")
            return
        }

        self.co2Characteristic = co2Characteristic
        self.temperatureCharacteristic = temperatureCharacteristic
        pendingNotificationUUIDs = [Self.co2UUID, Self.temperatureUUID]
        phase = .subscribing(displayName(for: peripheral))

        peripheral.setNotifyValue(true, for: co2Characteristic)
        peripheral.setNotifyValue(true, for: temperatureCharacteristic)
    }

    private func completeNotificationSubscription(for characteristic: CBCharacteristic) {
        pendingNotificationUUIDs.remove(characteristic.uuid)

        if pendingNotificationUUIDs.isEmpty {
            phase = .connected(displayName(for: sensorPeripheral))
        }
    }

    private func updateValue(from characteristic: CBCharacteristic) {
        guard let data = characteristic.value else {
            return
        }

        switch characteristic.uuid {
        case Self.co2UUID:
            guard let value = data.littleEndianUInt16 else {
                fail("Received an invalid CO2 payload.")
                return
            }
            co2PPM = value
            lastUpdate = Date()
        case Self.temperatureUUID:
            guard let value = data.littleEndianInt16 else {
                fail("Received an invalid temperature payload.")
                return
            }
            temperatureC = value
            lastUpdate = Date()
        default:
            break
        }
    }

    private func fail(_ message: String) {
        cancelScanTimeout()
        centralManager.stopScan()

        if let sensorPeripheral {
            pendingFailureMessage = message
            centralManager.cancelPeripheralConnection(sensorPeripheral)
            return
        }

        cleanupConnectionState()
        phase = .failed(message)
    }

    private func cleanupConnectionState() {
        sensorPeripheral?.delegate = nil
        sensorPeripheral = nil
        co2Characteristic = nil
        temperatureCharacteristic = nil
        pendingNotificationUUIDs.removeAll()
    }

    private func cancelScanTimeout() {
        scanTimeoutWorkItem?.cancel()
        scanTimeoutWorkItem = nil
    }

    private func displayName(for peripheral: CBPeripheral?, advertisedName: String? = nil) -> String {
        advertisedName ?? peripheral?.name ?? "RustCO2Sensor"
    }
}

extension SensorBLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            if shouldConnectWhenPoweredOn {
                scanForSensor()
            } else if phase == .waitingForBluetooth {
                phase = .idle
            }
        case .poweredOff:
            cancelScanTimeout()
            central.stopScan()
            cleanupConnectionState()
            phase = .bluetoothUnavailable("Bluetooth is off.")
        case .unauthorized:
            cleanupConnectionState()
            phase = .bluetoothUnavailable("Bluetooth permission is not granted.")
        case .unsupported:
            cleanupConnectionState()
            phase = .bluetoothUnavailable("This device does not support Bluetooth LE.")
        case .resetting:
            cancelScanTimeout()
            central.stopScan()
            cleanupConnectionState()
            phase = .waitingForBluetooth
        case .unknown:
            phase = .waitingForBluetooth
        @unknown default:
            cleanupConnectionState()
            phase = .bluetoothUnavailable("Bluetooth is unavailable.")
        }
    }

    func centralManager(
        _ central: CBCentralManager,
        didDiscover peripheral: CBPeripheral,
        advertisementData: [String: Any],
        rssi RSSI: NSNumber
    ) {
        let advertisedName = advertisementData[CBAdvertisementDataLocalNameKey] as? String
        connect(to: peripheral, advertisedName: advertisedName)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        phase = .discovering(displayName(for: peripheral))
        peripheral.discoverServices([Self.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        cleanupConnectionState()
        phase = .failed("Could not connect to \(displayName(for: peripheral)): \(error?.localizedDescription ?? "unknown error").")
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        cleanupConnectionState()

        if let pendingFailureMessage {
            phase = .failed(pendingFailureMessage)
        } else if isDisconnectingByUser {
            phase = .disconnected("Disconnected.")
        } else if let error {
            phase = .disconnected("Connection lost: \(error.localizedDescription)")
        } else {
            phase = .disconnected("Connection lost.")
        }

        isDisconnectingByUser = false
        pendingFailureMessage = nil
    }
}

extension SensorBLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error {
            fail("Could not discover services: \(error.localizedDescription)")
            return
        }

        discoverMeasurementCharacteristics(on: peripheral)
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let error {
            fail("Could not discover measurement characteristics: \(error.localizedDescription)")
            return
        }

        subscribeToMeasurements(on: peripheral, service: service)
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
        if let error {
            fail("Could not enable notifications for \(characteristic.uuid.uuidString): \(error.localizedDescription)")
            return
        }

        guard characteristic.isNotifying else {
            fail("Notifications are not enabled for \(characteristic.uuid.uuidString).")
            return
        }

        completeNotificationSubscription(for: characteristic)
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if let error {
            fail("Could not read notification value: \(error.localizedDescription)")
            return
        }

        updateValue(from: characteristic)
    }
}

extension SensorBLEManager {
    var statusTitle: String {
        phase.title
    }

    var statusDetail: String {
        phase.detail
    }

    var statusSystemImage: String {
        phase.systemImage
    }

    var statusColor: Color {
        phase.color
    }

    var primaryActionTitle: String {
        phase.primaryActionTitle
    }

    var primaryActionSystemImage: String {
        phase.primaryActionSystemImage
    }

    var isPrimaryActionEnabled: Bool {
        phase.isPrimaryActionEnabled
    }
}

enum ConnectionPhase: Equatable {
    case idle
    case waitingForBluetooth
    case bluetoothUnavailable(String)
    case scanning
    case connecting(String)
    case discovering(String)
    case subscribing(String)
    case connected(String)
    case disconnected(String)
    case failed(String)

    var title: String {
        switch self {
        case .idle:
            "Ready"
        case .waitingForBluetooth:
            "Waiting for Bluetooth"
        case .bluetoothUnavailable:
            "Bluetooth unavailable"
        case .scanning:
            "Scanning"
        case .connecting(let name):
            "Connecting to \(name)"
        case .discovering(let name):
            "Discovering \(name)"
        case .subscribing(let name):
            "Subscribing to \(name)"
        case .connected(let name):
            "Connected to \(name)"
        case .disconnected:
            "Disconnected"
        case .failed:
            "Connection failed"
        }
    }

    var detail: String {
        switch self {
        case .idle:
            "Tap Scan to connect to the sensor."
        case .waitingForBluetooth:
            "The app will scan when Bluetooth becomes available."
        case .bluetoothUnavailable(let message):
            message
        case .scanning:
            "Looking for RustCO2Sensor by its BLE service UUID."
        case .connecting:
            "Opening the BLE connection."
        case .discovering:
            "Finding the CO2 service and measurement characteristics."
        case .subscribing:
            "Enabling live CO2 and temperature notifications."
        case .connected:
            "Live notifications are active."
        case .disconnected(let message):
            message
        case .failed(let message):
            message
        }
    }

    var systemImage: String {
        switch self {
        case .connected:
            "checkmark.circle.fill"
        case .failed, .bluetoothUnavailable:
            "exclamationmark.triangle.fill"
        case .disconnected:
            "bolt.horizontal.circle.fill"
        case .idle:
            "antenna.radiowaves.left.and.right"
        case .waitingForBluetooth, .scanning, .connecting, .discovering, .subscribing:
            "dot.radiowaves.left.and.right"
        }
    }

    var color: Color {
        switch self {
        case .connected:
            .green
        case .failed, .bluetoothUnavailable:
            .orange
        case .disconnected:
            .red
        case .idle, .waitingForBluetooth, .scanning, .connecting, .discovering, .subscribing:
            .blue
        }
    }

    var primaryActionTitle: String {
        switch self {
        case .connected:
            "Disconnect"
        case .disconnected:
            "Reconnect"
        case .failed, .bluetoothUnavailable:
            "Retry"
        case .idle:
            "Scan"
        case .waitingForBluetooth:
            "Waiting"
        case .scanning:
            "Scanning"
        case .connecting, .discovering, .subscribing:
            "Connecting"
        }
    }

    var primaryActionSystemImage: String {
        switch self {
        case .connected:
            "xmark.circle"
        case .disconnected:
            "arrow.clockwise"
        case .failed, .bluetoothUnavailable:
            "arrow.clockwise"
        case .idle:
            "antenna.radiowaves.left.and.right"
        case .waitingForBluetooth, .scanning, .connecting, .discovering, .subscribing:
            "hourglass"
        }
    }

    var isPrimaryActionEnabled: Bool {
        switch self {
        case .waitingForBluetooth, .scanning, .connecting, .discovering, .subscribing:
            false
        case .idle, .bluetoothUnavailable, .connected, .disconnected, .failed:
            true
        }
    }
}

private extension Data {
    var littleEndianUInt16: UInt16? {
        guard count >= 2 else {
            return nil
        }

        return UInt16(self[startIndex]) | (UInt16(self[index(after: startIndex)]) << 8)
    }

    var littleEndianInt16: Int16? {
        guard let value = littleEndianUInt16 else {
            return nil
        }

        return Int16(bitPattern: value)
    }
}

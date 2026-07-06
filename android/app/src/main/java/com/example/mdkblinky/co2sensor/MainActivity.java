package com.example.mdkblinky.co2sensor;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.BluetoothStatusCodes;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanRecord;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import java.text.DateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Queue;
import java.util.UUID;

public final class MainActivity extends Activity {
    private static final UUID SERVICE_UUID =
            UUID.fromString("b22d7f14-9361-4309-932e-ffbdefed97fe");
    private static final UUID CO2_UUID =
            UUID.fromString("c7d0c8a8-db04-4199-869d-5f80091f2036");
    private static final UUID TEMPERATURE_UUID =
            UUID.fromString("c7d0c8a8-db04-4199-869d-5f80091f2037");
    private static final UUID CCCD_UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private static final int REQUEST_BLE_PERMISSIONS = 1001;
    private static final long SCAN_TIMEOUT_MS = 12_000L;

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final Queue<BluetoothGattCharacteristic> notificationQueue = new ArrayDeque<>();
    private final DateFormat timeFormatter = DateFormat.getTimeInstance(DateFormat.MEDIUM);

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLeScanner scanner;
    private ScanCallback scanCallback;
    private BluetoothGatt bluetoothGatt;
    private BluetoothGattCharacteristic co2Characteristic;
    private BluetoothGattCharacteristic temperatureCharacteristic;

    private Phase phase = Phase.IDLE;
    private boolean shouldStartAfterPermission;
    private boolean userDisconnecting;
    private String connectedName = "RustCO2Sensor";

    private TextView co2ValueView;
    private TextView temperatureValueView;
    private TextView statusTitleView;
    private TextView statusDetailView;
    private TextView lastUpdateView;
    private Button primaryButton;

    private final Runnable scanTimeoutRunnable = new Runnable() {
        @Override
        public void run() {
            if (phase == Phase.SCANNING) {
                stopScan();
                fail("No RustCO2Sensor advertising the CO2 service was found.");
            }
        }
    };

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    handleConnectionStateChange(gatt, status, newState);
                }
            });
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    handleServicesDiscovered(gatt, status);
                }
            });
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    if (status != BluetoothGatt.GATT_SUCCESS) {
                        fail("Could not enable notifications: GATT status " + status + ".");
                        return;
                    }
                    enableNextNotification();
                }
            });
        }

        @Override
        public void onCharacteristicChanged(
                BluetoothGatt gatt,
                BluetoothGattCharacteristic characteristic
        ) {
            byte[] value = characteristic.getValue();
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    handleNotification(characteristic.getUuid(), value);
                }
            });
        }

        @Override
        public void onCharacteristicChanged(
                BluetoothGatt gatt,
                BluetoothGattCharacteristic characteristic,
                byte[] value
        ) {
            byte[] copiedValue = value == null ? null : value.clone();
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    handleNotification(characteristic.getUuid(), copiedValue);
                }
            });
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager == null ? null : bluetoothManager.getAdapter();

        buildUi();
        setPhase(Phase.IDLE, "Tap Scan to connect to the sensor.");
    }

    @Override
    protected void onDestroy() {
        shouldStartAfterPermission = false;
        stopScan();
        closeGatt();
        super.onDestroy();
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode,
            String[] permissions,
            int[] grantResults
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode != REQUEST_BLE_PERMISSIONS) {
            return;
        }

        if (hasRequiredPermissions()) {
            if (shouldStartAfterPermission) {
                startScanFlow();
            } else {
                setPhase(Phase.IDLE, "Tap Scan to connect to the sensor.");
            }
            return;
        }

        setPhase(
                Phase.PERMISSION_REQUIRED,
                "Bluetooth permissions are needed to scan for and connect to RustCO2Sensor."
        );
    }

    private void buildUi() {
        ScrollView scrollView = new ScrollView(this);
        scrollView.setFillViewport(true);
        scrollView.setBackgroundColor(Color.rgb(244, 245, 247));

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setGravity(Gravity.CENTER_HORIZONTAL);
        int pagePadding = dp(24);
        root.setPadding(pagePadding, pagePadding, pagePadding, pagePadding);
        scrollView.addView(
                root,
                new ScrollView.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT
                )
        );

        co2ValueView = new TextView(this);
        co2ValueView.setText("--");
        co2ValueView.setGravity(Gravity.CENTER);
        co2ValueView.setTextColor(Color.rgb(26, 33, 38));
        co2ValueView.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        co2ValueView.setTextSize(86);
        co2ValueView.setSingleLine(true);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            co2ValueView.setAutoSizeTextTypeUniformWithConfiguration(
                    42,
                    86,
                    2,
                    TypedValue.COMPLEX_UNIT_SP
            );
        }
        LinearLayout.LayoutParams co2Params = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT
        );
        co2Params.topMargin = dp(28);
        root.addView(co2ValueView, co2Params);

        TextView co2LabelView = new TextView(this);
        co2LabelView.setText("ppm CO2");
        co2LabelView.setGravity(Gravity.CENTER);
        co2LabelView.setTextColor(Color.rgb(83, 96, 105));
        co2LabelView.setTextSize(20);
        co2LabelView.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        root.addView(
                co2LabelView,
                new LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT
                )
        );

        LinearLayout temperatureRow = createPanel();
        temperatureRow.setOrientation(LinearLayout.HORIZONTAL);
        temperatureRow.setGravity(Gravity.CENTER_VERTICAL);
        LinearLayout.LayoutParams panelParams = panelLayoutParams();
        panelParams.topMargin = dp(32);

        TextView temperatureTitleView = new TextView(this);
        temperatureTitleView.setText("Temperature");
        temperatureTitleView.setTextColor(Color.rgb(31, 40, 45));
        temperatureTitleView.setTextSize(18);
        temperatureTitleView.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        temperatureRow.addView(
                temperatureTitleView,
                new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1)
        );

        temperatureValueView = new TextView(this);
        temperatureValueView.setText("-- C");
        temperatureValueView.setTextColor(Color.rgb(31, 40, 45));
        temperatureValueView.setTextSize(24);
        temperatureValueView.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        temperatureRow.addView(
                temperatureValueView,
                new LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.WRAP_CONTENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT
                )
        );
        root.addView(temperatureRow, panelParams);

        LinearLayout statusPanel = createPanel();
        statusPanel.setOrientation(LinearLayout.VERTICAL);
        LinearLayout.LayoutParams statusParams = panelLayoutParams();
        statusParams.topMargin = dp(16);

        statusTitleView = new TextView(this);
        statusTitleView.setTextSize(18);
        statusTitleView.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        statusPanel.addView(
                statusTitleView,
                new LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT
                )
        );

        statusDetailView = new TextView(this);
        statusDetailView.setTextColor(Color.rgb(83, 96, 105));
        statusDetailView.setTextSize(15);
        statusDetailView.setPadding(0, dp(8), 0, 0);
        statusPanel.addView(
                statusDetailView,
                new LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT
                )
        );

        lastUpdateView = new TextView(this);
        lastUpdateView.setTextColor(Color.rgb(105, 117, 125));
        lastUpdateView.setTextSize(13);
        lastUpdateView.setVisibility(View.GONE);
        lastUpdateView.setPadding(0, dp(12), 0, 0);
        statusPanel.addView(
                lastUpdateView,
                new LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT
                )
        );
        root.addView(statusPanel, statusParams);

        primaryButton = new Button(this);
        primaryButton.setAllCaps(false);
        primaryButton.setTextSize(18);
        primaryButton.setMinHeight(dp(52));
        primaryButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                performPrimaryAction();
            }
        });
        LinearLayout.LayoutParams buttonParams = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                dp(56)
        );
        buttonParams.topMargin = dp(28);
        root.addView(primaryButton, buttonParams);

        setContentView(scrollView);
    }

    private LinearLayout createPanel() {
        LinearLayout panel = new LinearLayout(this);
        panel.setPadding(dp(18), dp(18), dp(18), dp(18));
        GradientDrawable background = new GradientDrawable();
        background.setColor(Color.WHITE);
        background.setCornerRadius(dp(8));
        panel.setBackground(background);
        return panel;
    }

    private LinearLayout.LayoutParams panelLayoutParams() {
        return new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT
        );
    }

    private void performPrimaryAction() {
        if (phase == Phase.CONNECTED) {
            disconnectByUser();
            return;
        }

        if (phase.isBusy()) {
            return;
        }

        startScanFlow();
    }

    private void startScanFlow() {
        shouldStartAfterPermission = true;
        userDisconnecting = false;

        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)
                || bluetoothAdapter == null) {
            setPhase(
                    Phase.BLUETOOTH_UNAVAILABLE,
                    "This device does not support Bluetooth LE."
            );
            return;
        }

        if (!hasRequiredPermissions()) {
            requestBlePermissions();
            return;
        }

        if (!isBluetoothEnabled()) {
            setPhase(Phase.BLUETOOTH_UNAVAILABLE, "Bluetooth is off.");
            return;
        }

        scanForSensor();
    }

    private void scanForSensor() {
        closeGatt();
        clearMeasurements();
        setPhase(Phase.SCANNING, "Looking for RustCO2Sensor by its BLE service UUID.");

        scanner = bluetoothAdapter.getBluetoothLeScanner();
        if (scanner == null) {
            fail("Bluetooth scanning is unavailable.");
            return;
        }

        scanCallback = new ScanCallback() {
            @Override
            public void onScanResult(int callbackType, ScanResult result) {
                handleScanResult(result);
            }

            @Override
            public void onBatchScanResults(List<ScanResult> results) {
                if (!results.isEmpty()) {
                    handleScanResult(results.get(0));
                }
            }

            @Override
            public void onScanFailed(int errorCode) {
                fail("BLE scan failed with error code " + errorCode + ".");
            }
        };

        ScanFilter filter = new ScanFilter.Builder()
                .setServiceUuid(new ParcelUuid(SERVICE_UUID))
                .build();
        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build();

        try {
            scanner.startScan(Collections.singletonList(filter), settings, scanCallback);
            mainHandler.removeCallbacks(scanTimeoutRunnable);
            mainHandler.postDelayed(scanTimeoutRunnable, SCAN_TIMEOUT_MS);
        } catch (SecurityException exception) {
            fail("Bluetooth permission is not granted.");
        }
    }

    private void handleScanResult(ScanResult result) {
        if (phase != Phase.SCANNING || result == null || result.getDevice() == null) {
            return;
        }

        ScanRecord scanRecord = result.getScanRecord();
        String advertisedName = scanRecord == null ? null : scanRecord.getDeviceName();
        connectToDevice(result.getDevice(), advertisedName);
    }

    private void connectToDevice(BluetoothDevice device, String advertisedName) {
        stopScan();
        connectedName = advertisedName == null || advertisedName.isEmpty()
                ? "RustCO2Sensor"
                : advertisedName;
        setPhase(Phase.CONNECTING, "Opening the BLE connection.");

        try {
            bluetoothGatt = device.connectGatt(
                    this,
                    false,
                    gattCallback,
                    BluetoothDevice.TRANSPORT_LE
            );
        } catch (SecurityException exception) {
            fail("Bluetooth permission is not granted.");
        }

        if (bluetoothGatt == null) {
            fail("Could not start a BLE connection to " + connectedName + ".");
        }
    }

    private void handleConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
        if (bluetoothGatt != null && gatt != bluetoothGatt) {
            return;
        }

        if (bluetoothGatt == null && (phase == Phase.DISCONNECTED || phase == Phase.FAILED)) {
            return;
        }

        if (newState == BluetoothProfile.STATE_CONNECTED && status == BluetoothGatt.GATT_SUCCESS) {
            bluetoothGatt = gatt;
            setPhase(
                    Phase.DISCOVERING,
                    "Finding the CO2 service and measurement characteristics."
            );
            try {
                if (!gatt.discoverServices()) {
                    fail("Could not start service discovery.");
                }
            } catch (SecurityException exception) {
                fail("Bluetooth permission is not granted.");
            }
            return;
        }

        if (newState == BluetoothProfile.STATE_DISCONNECTED) {
            boolean wasConnecting = phase == Phase.CONNECTING
                    || phase == Phase.DISCOVERING
                    || phase == Phase.SUBSCRIBING;
            closeGatt(gatt);

            if (userDisconnecting) {
                userDisconnecting = false;
                setPhase(Phase.DISCONNECTED, "Disconnected.");
            } else if (wasConnecting && status != BluetoothGatt.GATT_SUCCESS) {
                setPhase(
                        Phase.FAILED,
                        "Could not connect to " + connectedName + ": GATT status " + status + "."
                );
            } else if (status == BluetoothGatt.GATT_SUCCESS) {
                setPhase(Phase.DISCONNECTED, "Connection lost.");
            } else {
                setPhase(Phase.DISCONNECTED, "Connection lost: GATT status " + status + ".");
            }
        }
    }

    private void handleServicesDiscovered(BluetoothGatt gatt, int status) {
        if (status != BluetoothGatt.GATT_SUCCESS) {
            fail("Could not discover services: GATT status " + status + ".");
            return;
        }

        BluetoothGattService service = gatt.getService(SERVICE_UUID);
        if (service == null) {
            fail("The connected peripheral does not expose the CO2 service.");
            return;
        }

        co2Characteristic = service.getCharacteristic(CO2_UUID);
        if (co2Characteristic == null) {
            fail("The CO2 concentration characteristic is missing.");
            return;
        }

        temperatureCharacteristic = service.getCharacteristic(TEMPERATURE_UUID);
        if (temperatureCharacteristic == null) {
            fail("The temperature characteristic is missing.");
            return;
        }

        notificationQueue.clear();
        notificationQueue.add(co2Characteristic);
        notificationQueue.add(temperatureCharacteristic);
        setPhase(Phase.SUBSCRIBING, "Enabling live CO2 and temperature notifications.");
        enableNextNotification();
    }

    private void enableNextNotification() {
        if (bluetoothGatt == null) {
            fail("The BLE connection closed before notifications were enabled.");
            return;
        }

        BluetoothGattCharacteristic characteristic = notificationQueue.poll();
        if (characteristic == null) {
            setPhase(Phase.CONNECTED, "Live notifications are active.");
            return;
        }

        BluetoothGattDescriptor cccd = characteristic.getDescriptor(CCCD_UUID);
        if (cccd == null) {
            fail("The notification descriptor is missing for " + characteristic.getUuid() + ".");
            return;
        }

        try {
            if (!bluetoothGatt.setCharacteristicNotification(characteristic, true)) {
                fail("Could not enable local notifications for " + characteristic.getUuid() + ".");
                return;
            }

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                int result = bluetoothGatt.writeDescriptor(
                        cccd,
                        BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                );
                if (result != BluetoothStatusCodes.SUCCESS) {
                    fail("Could not write the notification descriptor: status " + result + ".");
                }
            } else {
                cccd.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                if (!bluetoothGatt.writeDescriptor(cccd)) {
                    fail("Could not write the notification descriptor.");
                }
            }
        } catch (SecurityException exception) {
            fail("Bluetooth permission is not granted.");
        }
    }

    private void handleNotification(UUID characteristicUuid, byte[] value) {
        if (CO2_UUID.equals(characteristicUuid)) {
            Integer ppm = decodeUInt16(value);
            if (ppm == null) {
                fail("Received an invalid CO2 payload.");
                return;
            }
            co2ValueView.setText(String.valueOf(ppm));
            updateLastUpdate();
            return;
        }

        if (TEMPERATURE_UUID.equals(characteristicUuid)) {
            Integer temperatureC = decodeInt16(value);
            if (temperatureC == null) {
                fail("Received an invalid temperature payload.");
                return;
            }
            temperatureValueView.setText(temperatureC + " C");
            updateLastUpdate();
        }
    }

    private Integer decodeUInt16(byte[] value) {
        if (value == null || value.length < 2) {
            return null;
        }

        return (value[0] & 0xff) | ((value[1] & 0xff) << 8);
    }

    private Integer decodeInt16(byte[] value) {
        Integer raw = decodeUInt16(value);
        if (raw == null) {
            return null;
        }

        return raw >= 0x8000 ? raw - 0x10000 : raw;
    }

    private void updateLastUpdate() {
        lastUpdateView.setText("Last update " + timeFormatter.format(new Date()));
        lastUpdateView.setVisibility(View.VISIBLE);
    }

    private void disconnectByUser() {
        shouldStartAfterPermission = false;
        userDisconnecting = true;
        stopScan();

        if (bluetoothGatt == null) {
            userDisconnecting = false;
            setPhase(Phase.DISCONNECTED, "Disconnected.");
            return;
        }

        try {
            bluetoothGatt.disconnect();
        } catch (SecurityException exception) {
            fail("Bluetooth permission is not granted.");
            return;
        }

        closeGatt();
        setPhase(Phase.DISCONNECTED, "Disconnected.");
    }

    private void fail(String message) {
        shouldStartAfterPermission = false;
        userDisconnecting = false;
        stopScan();
        closeGatt();
        setPhase(Phase.FAILED, message);
    }

    private void stopScan() {
        mainHandler.removeCallbacks(scanTimeoutRunnable);

        if (scanner != null && scanCallback != null) {
            try {
                scanner.stopScan(scanCallback);
            } catch (SecurityException ignored) {
                // The UI will surface permission problems through the next active operation.
            }
        }

        scanCallback = null;
        scanner = null;
    }

    private void closeGatt() {
        closeGatt(bluetoothGatt);
    }

    private void closeGatt(BluetoothGatt gatt) {
        notificationQueue.clear();
        co2Characteristic = null;
        temperatureCharacteristic = null;

        if (gatt == null) {
            return;
        }

        if (gatt == bluetoothGatt) {
            bluetoothGatt = null;
        }

        gatt.close();
    }

    private void clearMeasurements() {
        co2ValueView.setText("--");
        temperatureValueView.setText("-- C");
        lastUpdateView.setVisibility(View.GONE);
    }

    private boolean isBluetoothEnabled() {
        try {
            return bluetoothAdapter != null && bluetoothAdapter.isEnabled();
        } catch (SecurityException exception) {
            return false;
        }
    }

    private boolean hasRequiredPermissions() {
        for (String permission : requiredPermissions()) {
            if (checkSelfPermission(permission) != PackageManager.PERMISSION_GRANTED) {
                return false;
            }
        }
        return true;
    }

    private void requestBlePermissions() {
        List<String> permissions = requiredPermissions();
        setPhase(
                Phase.PERMISSION_REQUIRED,
                "Bluetooth permissions are needed to scan for and connect to RustCO2Sensor."
        );
        requestPermissions(permissions.toArray(new String[0]), REQUEST_BLE_PERMISSIONS);
    }

    private List<String> requiredPermissions() {
        List<String> permissions = new ArrayList<>();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN);
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT);
        } else {
            permissions.add(Manifest.permission.ACCESS_FINE_LOCATION);
        }
        return permissions;
    }

    private void setPhase(Phase newPhase, String detail) {
        phase = newPhase;
        statusTitleView.setText(newPhase.title(connectedName));
        statusTitleView.setTextColor(newPhase.color());
        statusDetailView.setText(detail);
        primaryButton.setText(newPhase.primaryActionTitle());
        primaryButton.setEnabled(!newPhase.isBusy());
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private enum Phase {
        IDLE,
        PERMISSION_REQUIRED,
        BLUETOOTH_UNAVAILABLE,
        SCANNING,
        CONNECTING,
        DISCOVERING,
        SUBSCRIBING,
        CONNECTED,
        DISCONNECTED,
        FAILED;

        String title(String name) {
            switch (this) {
                case IDLE:
                    return "Ready";
                case PERMISSION_REQUIRED:
                    return "Bluetooth permission needed";
                case BLUETOOTH_UNAVAILABLE:
                    return "Bluetooth unavailable";
                case SCANNING:
                    return "Scanning";
                case CONNECTING:
                    return "Connecting to " + name;
                case DISCOVERING:
                    return "Discovering " + name;
                case SUBSCRIBING:
                    return "Subscribing to " + name;
                case CONNECTED:
                    return "Connected to " + name;
                case DISCONNECTED:
                    return "Disconnected";
                case FAILED:
                    return "Connection failed";
                default:
                    return "";
            }
        }

        String primaryActionTitle() {
            switch (this) {
                case CONNECTED:
                    return "Disconnect";
                case DISCONNECTED:
                    return "Reconnect";
                case FAILED:
                case BLUETOOTH_UNAVAILABLE:
                case PERMISSION_REQUIRED:
                    return "Retry";
                case SCANNING:
                    return "Scanning";
                case CONNECTING:
                case DISCOVERING:
                case SUBSCRIBING:
                    return "Connecting";
                case IDLE:
                default:
                    return "Scan";
            }
        }

        boolean isBusy() {
            return this == SCANNING
                    || this == CONNECTING
                    || this == DISCOVERING
                    || this == SUBSCRIBING;
        }

        int color() {
            switch (this) {
                case CONNECTED:
                    return Color.rgb(36, 128, 82);
                case FAILED:
                case BLUETOOTH_UNAVAILABLE:
                case PERMISSION_REQUIRED:
                    return Color.rgb(186, 98, 24);
                case DISCONNECTED:
                    return Color.rgb(169, 55, 55);
                case IDLE:
                case SCANNING:
                case CONNECTING:
                case DISCOVERING:
                case SUBSCRIBING:
                default:
                    return Color.rgb(45, 103, 172);
            }
        }
    }
}

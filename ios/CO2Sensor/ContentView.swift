import SwiftUI
#if os(iOS)
import UIKit
#elseif os(macOS)
import AppKit
#endif

struct ContentView: View {
    @ObservedObject var manager: SensorBLEManager

    var body: some View {
        VStack(spacing: 28) {
            Spacer(minLength: 24)

            VStack(spacing: 8) {
                Text(co2Text)
                    .font(.system(size: 86, weight: .bold, design: .rounded))
                    .monospacedDigit()
                    .minimumScaleFactor(0.55)
                    .lineLimit(1)

                Text("ppm CO2")
                    .font(.title3.weight(.semibold))
                    .foregroundStyle(.secondary)
            }
            .frame(maxWidth: .infinity)

            VStack(spacing: 16) {
                metricRow(
                    title: "Temperature",
                    value: temperatureText,
                    systemImage: "thermometer.medium"
                )

                statusPanel
            }

            Spacer(minLength: 16)

            Button(action: manager.performPrimaryAction) {
                Label(manager.primaryActionTitle, systemImage: manager.primaryActionSystemImage)
                    .font(.headline)
                    .frame(maxWidth: .infinity)
                    .frame(height: 52)
            }
            .buttonStyle(.borderedProminent)
            .disabled(!manager.isPrimaryActionEnabled)
        }
        .padding(24)
        .background(Color.appGroupedBackground)
    }

    private var statusPanel: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(spacing: 10) {
                Image(systemName: manager.statusSystemImage)
                    .foregroundStyle(manager.statusColor)
                    .frame(width: 24)

                Text(manager.statusTitle)
                    .font(.headline)
                    .foregroundStyle(.primary)

                Spacer()
            }

            Text(manager.statusDetail)
                .font(.subheadline)
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)

            if let lastUpdate = manager.lastUpdate {
                Text("Last update \(lastUpdate.formatted(date: .omitted, time: .standard))")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
        .padding(18)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color.appSecondaryGroupedBackground)
        .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
    }

    private var co2Text: String {
        guard let co2PPM = manager.co2PPM else {
            return "--"
        }
        return "\(co2PPM)"
    }

    private var temperatureText: String {
        guard let temperatureC = manager.temperatureC else {
            return "-- C"
        }
        return "\(temperatureC) C"
    }

    private func metricRow(title: String, value: String, systemImage: String) -> some View {
        HStack(spacing: 14) {
            Image(systemName: systemImage)
                .font(.title3)
                .foregroundStyle(.secondary)
                .frame(width: 28)

            Text(title)
                .font(.headline)

            Spacer()

            Text(value)
                .font(.title3.weight(.semibold))
                .monospacedDigit()
        }
        .padding(18)
        .frame(maxWidth: .infinity)
        .background(Color.appSecondaryGroupedBackground)
        .clipShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
    }
}

private extension Color {
    static var appGroupedBackground: Color {
        #if os(iOS)
        Color(uiColor: .systemGroupedBackground)
        #elseif os(macOS)
        Color(nsColor: .windowBackgroundColor)
        #else
        Color.gray.opacity(0.12)
        #endif
    }

    static var appSecondaryGroupedBackground: Color {
        #if os(iOS)
        Color(uiColor: .secondarySystemGroupedBackground)
        #elseif os(macOS)
        Color(nsColor: .controlBackgroundColor)
        #else
        Color.gray.opacity(0.08)
        #endif
    }
}

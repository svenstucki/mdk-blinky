import SwiftUI

@main
struct CO2SensorApp: App {
    @StateObject private var bleManager = SensorBLEManager()

    var body: some Scene {
        WindowGroup {
            ContentView(manager: bleManager)
                .onAppear {
                    bleManager.start()
                }
        }
    }
}

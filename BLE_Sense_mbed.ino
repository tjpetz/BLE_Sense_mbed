/*
 * Use MBedOS to expose the Nano 33 BLE Sense Temperature, Humidity, and Pressure
 * sensors via the standard MBedOS BLE services.
 * 
 * This version advertises with CODED_PHY for BLE 5.0 to support a longer range, ideally
 * providing whole house coverage with just a single Central device (i.e Raspberry Pi)
 * 
 */

#include <mbed.h>
#include <events/mbed_events.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/BatteryService.h"
#include "ble/services/EnvironmentalService.h"

#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>

static mbed::DigitalOut led1(LED1, 0);

static events::EventQueue eventQueue(16 * EVENTS_EVENT_SIZE);
const static char deviceName[] = "EnvironSensor";

const static ble::phy_set_t codedPHY(ble::phy_t::LE_CODED);

void printError(ble_error_t err, const char* msg) {
  Serial.print("Error: ");
  Serial.print(err);
  Serial.print(" -- ");
  Serial.println(msg);
}

class BLENanoSenseTransmitter : ble::Gap::EventHandler {
  public:
    BLENanoSenseTransmitter(BLE &ble, const char* name, events::EventQueue &eventQueue)
      : ble_(ble), deviceName_(name), eventQueue_(eventQueue),
        // battery_uuid_(GattService::UUID_BATTERY_SERVICE), batteryLevel_(100),
        // batteryService_(ble, batteryLevel_),
        environmentalService_uuid_(GattService::UUID_ENVIRONMENTAL_SERVICE),
        environmentalService_(ble),
        // temperature_(20), humidity_(50), pressure_(1000),
        advDataBuilder(advBuffer)
      {}

    void start() {
      ble_.gap().setEventHandler(this);
      ble_.init(this, &BLENanoSenseTransmitter::onInitComplete);
      eventQueue_.call_every(5000, this, &BLENanoSenseTransmitter::updateEnvironmentMeasurement);
      eventQueue_.dispatch_forever();
    }

  private:
    void onInitComplete(BLE::InitializationCompleteCallbackContext *params) {
      if (params->error != BLE_ERROR_NONE) {
        printError(params->error, "BLE initialization failed.");
        return;
      }

      Serial.println("BLE initialization complete!");

      if (ble_.gap().isFeatureSupported(ble::controller_supported_features_t::LE_2M_PHY)) {
        Serial.println("LE_2M_PHY supported");
      } else {
        Serial.println("LE_2M_PHY NOT supported");
      }

      if (ble_.gap().isFeatureSupported(ble::controller_supported_features_t::LE_CODED_PHY)) {
        Serial.println("LE_CODED_PHY supported");
      } else {
        Serial.println("LE_CODED_PHY NOT supported");
      }

      if (ble_.gap().isFeatureSupported(ble::controller_supported_features_t::LE_EXTENDED_ADVERTISING)) {
        Serial.println("LE_EXTENDED_ADVERTISING supported");
      } else {
        Serial.println("LE_EXTENDED_ADVERTISING NOT supported");
      }

      // Set the prefered PHY for connections (this is different than for advertising)
      ble_error_t error = ble_.gap().setPreferredPhys(&codedPHY, &codedPHY);
      if (error) {
        printError(error, "Error setting prefered PHY");
      }
      
      startAdvertising();
    }

    void startAdvertising() {
      advDataBuilder.setFlags();
      advDataBuilder.setLocalServiceList(mbed::make_Span(&environmentalService_uuid_, 1));
      advDataBuilder.setName(deviceName_);
      advDataBuilder.setTxPowerAdvertised(8);

      Serial.println("Setting up advertising...");
      
      ble_error_t error = ble_.gap().setAdvertisingParameters(
        ble::LEGACY_ADVERTISING_HANDLE,
        ble::AdvertisingParameters()
          .setPhy(ble::phy_t::LE_CODED, ble::phy_t::LE_CODED)
//          .setPhy(ble::phy_t::LE_1M, ble::phy_t::LE_2M)
          .setOwnAddressType(ble::own_address_type_t::PUBLIC)
          .setTxPower(8)
          .includeTxPowerInHeader(true)
          );

       if (error) {
          printError(error, "ble_.gap().setAdvertisingParameters failed!");
          return;
        } else {
          Serial.println("ble_.gap().setAdvertisingParameters succeeded");
        }
  
        error = ble_.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE, advDataBuilder.getAdvertisingData());
    
        if (error) {
          printError(error, "_ble.gap().setAdvertisingPayload() failed");
          return;
        }
    
        error = ble_.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    
        if (error) {
          printError(error, "_ble.gap().startAdvertising() failed");
          return;
        } else {
          Serial.println("ble_.gap().startAdvertising() started.");
        }
    }

    void updateEnvironmentMeasurement() {
      // Read the sensor and update the BLE environment characteristics
      environmentalService_.updateTemperature(HTS.readTemperature());
      environmentalService_.updateHumidity(HTS.readHumidity());
      environmentalService_.updatePressure((uint32_t)(BARO.readPressure()));
    }
  
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent & a) override {
      Serial.println("Disconnected");
      ble_.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
      led1 = 0;
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent & a) override {
      Serial.println("Connecting!");
      led1 = 1;
      // Update the environment measurements

      // Request the phy information
      ble_error_t error = ble_.gap().readPhy(a.getConnectionHandle());
      if (error) {
        Serial.println("Error initiating a ReadPHY!");
        Serial.print(error); Serial.println(BLE::errorToString(error));
      } else {
        Serial.println("Requested ReadPHY");
      }
    }

    void onReadPhy(ble_error_t status, ble::connection_handle_t connectionHandle, ble::phy_t txPhy, ble::phy_t rxPhy) override {
      Serial.println("Processing onReadPhy() event");
      if (status) {
        Serial.print("Error: "); Serial.println(BLE::errorToString(status)); 
      } else {
        Serial.print("Tx PHY = "); Serial.println(txPhy.value());
        Serial.print("Rx PHY = "); Serial.println(rxPhy.value());
      }
    }
    
    void onAdvertisingEnd(const ble::AdvertisingEndEvent & a) override {
      Serial.println("Restarting advertising");
      ble_.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    }

    void onUpdateConnectionParametersRequest(const ble::UpdateConnectionParametersRequestEvent & a) override {
      Serial.println("Received request to update connection parameters");
    }

    void onScanRequestReceived(const ble::ScanRequestEvent& event) override {
      Serial.println("Scan request received");
    }

    void onPhyUpdateComplete(ble_error_t status, ble::connection_handle_t connectionHandle, ble::phy_t txPhy, ble::phy_t rxPhy) override {
      Serial.println("Phy Update Complete");
      Serial.print("Tx PHY = "); Serial.println(txPhy.value());
      Serial.print("Rx PHY = "); Serial.println(rxPhy.value());
    }

    BLE &ble_;
    const char *deviceName_;
    events::EventQueue &eventQueue_;
    UUID environmentalService_uuid_;
    EnvironmentalService environmentalService_;

    uint8_t advBuffer[1000];
    ble::AdvertisingDataBuilder advDataBuilder;
    ble::advertising_handle_t adv_handle_;
};


void scheduleBleEvents(BLE::OnEventsToProcessCallbackContext *context) {
  eventQueue.call(mbed::Callback<void()>(&context->ble, &BLE::processEvents));  
}


void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Starting...");
  
  // Initialize the environment sensors.
  HTS.begin();
  BARO.begin();

  BLE &ble = BLE::Instance();
  ble.onEventsToProcess(scheduleBleEvents);

  BLENanoSenseTransmitter transmitter(ble, deviceName, eventQueue);
  transmitter.start();
}

void loop() {
}

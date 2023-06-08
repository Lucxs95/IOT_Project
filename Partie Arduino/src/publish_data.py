import paho.mqtt.publish as publish
import json
import time

mqtt_broker = "mqtt.eclipseprojects.io"
mqtt_port = 1883

def publish_data(idu, idswp, lat, lon):
    # Build the payload message
    payload = {
        "lat": lat,
        "lon": lon,
        "idu": idu,
        "idswp": idswp
    }
    payload_json = json.dumps(payload)

    # Publish the payload to the MQTT topic
    mqtt_topic = f"uca/waterbnb/{idu}/{idswp}"
    publish.single(mqtt_topic, payload_json, hostname=mqtt_broker, port=mqtt_port)

# Prompt for input
idu = "21904022" #input("Enter idu: ")
idswp = "lucasPool" #input("Enter idswp: ")
lat = 43.5704187 #input("Enter latitude: ")
lon = 6.9917197 #input("Enter longitude: ")

while True:
    # Publish the data to MQTT
    publish_data(idu, idswp, lat, lon)
    print("Position published")

    # Wait for 5 seconds
    time.sleep(5)

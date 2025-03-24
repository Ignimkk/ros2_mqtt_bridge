# import paho.mqtt.client as mqtt
# import json

# def on_connect(client, userdata, flags, rc):
#     print("Connected to MQTT Broker!")
#     client.subscribe("ros2/joint_states")

# def on_message(client, userdata, msg):
#     data = json.loads(msg.payload.decode("utf-8"))
#     print(f"Received Data: {json.dumps(data, indent=4)}")

# client = mqtt.Client()
# client.on_connect = on_connect
# client.on_message = on_message

# client.connect("localhost", 1883, 60) 
# client.loop_forever()


############################
import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker!")
    client.subscribe("ros/joint_states")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))  
        sec_value = data["header"]["stamp"]["sec"]  
        print(f"subscribe sec: {sec_value}")
    except (KeyError, json.JSONDecodeError) as e:
        print(f"Error processing message: {e}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60) 
client.loop_forever()

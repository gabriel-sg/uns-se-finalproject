import pyrebase
import json

PANID = 0xcafe
ADDR_NODO_1 = 0x6001
ADDR_NODO_2 = 0x6002
ADDR_NODO_3 = 0x6003

config = {
  "apiKey": "AIzaSyAeMsHJsZrkKdaRD_8dqPS8dkHbK6v9d6I",
  "authDomain": "redes-de-sensores.firebaseapp.com",
  "databaseURL": "https://redes-de-sensores.firebaseio.com",
  "projectId": "redes-de-sensores",
  "storageBucket": "redes-de-sensores.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

# data = {
#     format(hex(ADDR_NODO_1)): {
#         "PanId": format(hex(PANID)),
#         "Sensores": {
#             "Sensor1": {
#                 "Nombre": "Mortimer",
#                 "AtribType": "INT",
#                 "AtribVal": 1,
#             },
#             "Sensor2": {
#                 "Nombre": "Morty",
#                 "AtribType": "FLOAT",
#                 "AtribVal": 1.12,
#             },
#             "Sensor3": {
#                 "Nombre": "Smith",
#                 "AtribType": "STRING",
#                 "AtribVal": "Hola Cat",
#             },
#         },
#         "Actuadores": {
#             "Actuador1": {
#                 "Nombre": "Mortiro",
#                 "Estado": "OFF",
#             },
#             "Actuador2": {
#                 "Nombre": "Mortysanto",
#                 "Estado": "OFF",
#             },
#             "Actuador3": {
#                 "Nombre": "Smithty",
#                 "Estado": "OFF",
#             },
#         },
#     },
        
#     format(hex(ADDR_NODO_2)): {
#         "PanId": format(hex(PANID)),
#         "Sensores": {
#             "Sensor1": {
#                 "Nombre": "Lux",
#                 "AtribType": "INT",
#                 "AtribVal": 2,
#             },
#             "Sensor2": {
#                 "Nombre": "Temp",
#                 "AtribType": "INT",
#                 "AtribVal": 3,
#             },
#             "Sensor3": {
#                 "Nombre": "Humedad",
#                 "AtribType": "INT",
#                 "AtribVal": 30,
#             },
#         },
#         "Actuadores": {
#             "Actuador1": {
#                 "Nombre": "Foco",
#                 "Estado": "OFF",
#             },
#             "Actuador2": {
#                 "Nombre": "Servo",
#                 "Estado": "OFF",
#             },
#             "Actuador3": {
#                 "Nombre": "Buzzer",
#                 "Estado": "OFF",
#             },
#         },
#     },
    
#     format(hex(ADDR_NODO_3)): {
#         "PanId": format(hex(PANID)),
#         "Sensores": {
#             "Sensor1": {
#                 "Nombre": "PushBtn",
#                 "AtribType": "INT",
#                 "AtribVal": 30,
#             },
#             "Sensor2Id": {
#                 "Nombre": "---",
#                 "AtribType": "INT",
#                 "AtribVal": 30,
#             },
#             "Sensor3Id": {
#                 "Nombre": "Volt",
#                 "AtribType": "INT",
#                 "AtribVal": 30,
#             },
#         },
#         "Actuadores": {
#             "Actuador1Id": {
#                 "Nombre": "BtnOK",
#                 "Estado": "OFF"
#             },
#             "Actuador2Id": {
#                 "Nombre": "LED",
#                 "Estado": "OFF"
#             },
#             "Actuador3Id": {
#                 "Nombre": "7-Segs",
#                 "Estado": "OFF"
#             }
#         }
#     }
# }

# db.set(data)

print("########## All nodes ##########")
all_nodes = db.child(format(hex(ADDR_NODO_1))).child("Sensores").get()
# print(type(all_nodes))
for node in all_nodes.each():
    print(node.key())
    print(node.val())
    if (type(node.val()) is dict):
        for inner_node in node.val():
            # print(inner_node.key())
            print(inner_node)

print("########## JSON ##########")
data_encoded = json.loads(all_nodes)
print(data_encoded)

print("Fin ejecucion")

class Nodo:

    def __init__(self):
        
        self.node_id = ADDR_NODO_1
        self.panid = PANID
        self.actuadores = []
        for i in range(0,3):
            self.actuadores.append(0)
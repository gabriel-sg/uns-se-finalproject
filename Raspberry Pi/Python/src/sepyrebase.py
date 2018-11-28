import pyrebase

PANID = "0xcafe"
ADDR_NODO_1 = "0x6001"
ADDR_NODO_2 = "0x6002"
ADDR_NODO_3 = "0x6003"

ACTUADORES = "Actuadores"
ACT1 = "Actuador1"
ACT2 = "Actuador2"
ACT3 = "Actuador3"
ACT4 = "Actuador4"

SENSORES = "Sensores"
SEN1 = "Sensor1"
SEN2 = "Sensor2"
SEN3 = "Sensor3"
SEN4 = "Sensor4"

config = {
  "apiKey": "AIzaSyAeMsHJsZrkKdaRD_8dqPS8dkHbK6v9d6I",
  "authDomain": "redes-de-sensores.firebaseapp.com",
  "databaseURL": "https://redes-de-sensores.firebaseio.com",
  "projectId": "redes-de-sensores",
  "storageBucket": "redes-de-sensores.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

class Mensaje:

    def __init__(self,n_addr,s_id,val_type,val_val):
        
        self.pan_id = PANID
        self.node_id = n_addr
        if (s_id == 1):
            self.sen_id = SEN1
        elif (s_id == 2):
            self.sen_id = SEN2
        elif (s_id == 3):
            self.sen_id = SEN3
        elif (s_id == 4):
            self.sen_id = SEN4
        self.atrib_type = val_type
        self.atrib_val = val_val

    def send_sens_val(self):
        db.child(self.node_id).child(SENSORES).child(self.sen_id).update({"Val":self.atrib_val})
        # print("Valor enviado: {} {} {}".format(self.node_id,self.sen_id,self.atrib_val))

    # def send_act_val(self):
    #     db.child(self.node_id).child(ACTUADORES).child(self.act_id).update({"Val":self.atrib_val})

    # def retrieve_sens_val(self):
    #     d = db.child(self.node_id).child(SENSORES).child(self.sen_id).child("Val").get()
    #     return d.val()

    # def retrieve_act_val(self):
    #     d = db.child(self.node_id).child(ACTUADORES).child(self.act_id).child("Val").get()
    #     return d.val()

def set_actuador_callback(nodoId, atcuadorId, callback):
    global stream
    stream = db.child(nodoId).child(ACTUADORES).child(atcuadorId).child("Estado").stream(callback)
    # stream.start_stream()

def close_db():
    stream.close()


# data = {
#     ADDR_NODO_1: {
#         "PanId": PANID,
#         "Sensores": {
#             "Sensor1": {
#                 "Nombre": "Mortimer",
#                 "Tipo": "INT",
#                 "Val": 1,
#             },
#             "Sensor2": {
#                 "Nombre": "Morty",
#                 "Tipo": "FLOAT",
#                 "Val": 1.12,
#             },
#             "Sensor3": {
#                 "Nombre": "Smith",
#                 "Tipo": "STRING",
#                 "Val": "Hola Cat",
#             },
#             "Sensor4": {
#                 "Nombre": "Smith2",
#                 "Tipo": "STRING",
#                 "Val": "Hola",
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
#             "Actuador4": {
#                 "Nombre": "Smithty2",
#                 "Estado": "OFF",
#             },
#         },
#     },
        
#     ADDR_NODO_2: {
#         "PanId": PANID,
#         "Sensores": {
#             "Sensor1": {
#                 "Nombre": "Lux",
#                 "Tipo": "INT",
#                 "Val": 2,
#             },
#             "Sensor2": {
#                 "Nombre": "Temp",
#                 "Tipo": "INT",
#                 "Val": 3,
#             },
#             "Sensor3": {
#                 "Nombre": "Humedad",
#                 "Tipo": "INT",
#                 "Val": 30,
#             },
#             "Sensor4": {
#                 "Nombre": "Humedad2",
#                 "Tipo": "INT",
#                 "Val": 37,
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
#             "Actuador4": {
#                 "Nombre": "Buzzer2",
#                 "Estado": "OFF",
#             },
#         },
#     },
    
#     ADDR_NODO_3: {
#         "PanId": PANID,
#         "Sensores": {
#             "Sensor1": {
#                 "Nombre": "PushBtn",
#                 "Tipo": "INT",
#                 "Val": 30,
#             },
#             "Sensor2": {
#                 "Nombre": "---",
#                 "Tipo": "INT",
#                 "Val": 30,
#             },
#             "Sensor3": {
#                 "Nombre": "Volt",
#                 "Tipo": "INT",
#                 "Val": 30,
#             },
#             "Sensor4": {
#                 "Nombre": "Volt2",
#                 "Tipo": "INT",
#                 "Val": 37,
#             },
#         },
#         "Actuadores": {
#             "Actuador1": {
#                 "Nombre": "BtnOK",
#                 "Estado": "OFF"
#             },
#             "Actuador2": {
#                 "Nombre": "LED",
#                 "Estado": "OFF"
#             },
#             "Actuador3": {
#                 "Nombre": "7-Segs",
#                 "Estado": "OFF"
#             }
#             "Actuador4": {
#                 "Nombre": "7-Segs2",
#                 "Estado": "OFF"
#             }
#         }
#     }
# }

# db.set(data)

# print("########## All nodes ##########")
# all_nodes = db.child(ADDR_NODO_1).child("Sensores").get()
# print(type(all_nodes))
# for node in all_nodes.each():
#     print(node.key())
#     print(node.val())
#     if (type(node.val()) is dict):
#         for inner_node in node.val():
#             # print(inner_node.key())
#             print(inner_node)

# nuevo_valor = 30
# db.child(ADDR_NODO_1).child(SENSORES).child(SEN1).update({"Val":nuevo_valor})

# dato = db.child(format(hex(ADDR_NODO_1))).child("Sensores").child("Sensor1").child("Val").get()
# dato.val() = nuevo_valor

# print("Valor actualizado")

# dato = db.child(ADDR_NODO_1).child(SENSORES).child(SEN1).child("Nombre").get()
# print("Recupere el dato: ",dato.val())

# print("Fin carga módulo")

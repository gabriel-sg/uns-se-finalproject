#!/usr/bin/env python

# import sys
# import os
import threading
import wiringpi
import time
import mrf24j40
import sepyrebase
import queue

PANID = 0xcafe
ADDR_NODO_1 = 0x6001
ADDR_NODO_2 = 0x6002
ADDR_NODO_3 = 0x6003

###### Init wiringPi lib ######
wiringpi.wiringPiSetup() # wiringpi pin's number

###### Raspberry Pi GPIO ######
pin_reset = 27
pin_cs = 26
pin_interrupt = 28

############ Módulo MRF24J40 ############
# Crea objeto del módulo MRF
mrf = mrf24j40.Mrf24j(pin_reset, pin_cs, pin_interrupt)

###### Test Functions ######
run_test = 0

totalRead = 0
totalReadSucc = 0
totalWrite = 0
totalWriteSucc = 0

def testRW(printResults):
    init_time = 0
    contReadReg = 9
    contReadSucc = 0
    if (printResults):
        init_time = int(round(time.time() * 1000))
        print("\nLeyendo " + str(contReadReg) + " registros...\n")

    contReadSucc += assertReadShort("MRF_PACON2", mrf24j40.MRF_PACON2, 0b10001000, printResults)
    contReadSucc += assertReadShort("MRF_TXSTBL", mrf24j40.MRF_TXSTBL, 0b01110101, printResults)
    contReadSucc += assertReadShort("MRF_TXBCON1", mrf24j40.MRF_TXBCON1, 0b00110000, printResults)
    contReadSucc += assertReadLong("MRF_WAKETIMEL", mrf24j40.MRF_WAKETIMEL, 0b00001010, printResults)
    contReadSucc += assertReadShort("MRF_BBREG2", mrf24j40.MRF_BBREG2, 0b01001000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG3", mrf24j40.MRF_BBREG3, 0b11011000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG4", mrf24j40.MRF_BBREG4, 0b10011100, printResults)
    contReadSucc += assertReadShort("MRF_BBREG6", mrf24j40.MRF_BBREG6, 0b00000001, printResults)
    contReadSucc += assertReadShort("MRF_INTCON", mrf24j40.MRF_INTCON, 0b11111111, printResults)

    contWriteReg = 11
    contWriteSucc = 0
    if (printResults):
        print("\nEscribiendo " + str(contWriteReg) + " registros...\n")

    contWriteSucc += assertWriteShort("MRF_PACON2", mrf24j40.MRF_PACON2, 0x98, printResults)
    contWriteSucc += assertWriteShort("MRF_TXSTBL", mrf24j40.MRF_TXSTBL, 0x95, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON0", mrf24j40.MRF_RFCON0, 0x03, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON1", mrf24j40.MRF_RFCON1, 0x01, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON2", mrf24j40.MRF_RFCON2, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON6", mrf24j40.MRF_RFCON6, 0x90, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON7", mrf24j40.MRF_RFCON7, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON8", mrf24j40.MRF_RFCON8, 0x10, printResults)
    contWriteSucc += assertWriteLong("MRF_SLPCON1", mrf24j40.MRF_SLPCON1, 0x21, printResults)
    contWriteSucc += assertWriteShort("MRF_BBREG2", mrf24j40.MRF_BBREG2, 0x80, printResults)
    contWriteSucc += assertWriteShort("MRF_CCAEDTH", mrf24j40.MRF_CCAEDTH, 0x60, printResults)

    if (printResults):
        print("\n------------------Resumen------------------\n")
        print("Lecturas exitosas: " + str(contReadSucc))
        print("Lecturas fallidas: " + str(contReadReg - contReadSucc))
        print("\n")
        print("Escrituras exitosas: " + str(contWriteSucc))
        print("Escrituras fallidas: " + str(contWriteReg - contWriteSucc))
        print("\n")
        init_time = int(round(time.time() * 1000)) - init_time
        print("Test duration: " + str(init_time) + " ms\n")
        print("-------------------------------------------")

    global totalRead, totalReadSucc, totalWrite, totalWriteSucc
    totalRead += contReadReg
    totalReadSucc += contReadSucc
    totalWrite += contWriteReg
    totalWriteSucc += contWriteSucc

def assertWriteShort(name, address, toWriteValue, printResults):
    mrf.write_short(address, toWriteValue)
    readValue = mrf.read_short(address)
    if (printResults):
        print(str(name) + ": write value: " + str(toWriteValue) + ", read value: " + str(readValue))
    return toWriteValue == readValue

def assertWriteLong(name, address, toWriteValue, printResults):
    mrf.write_long(address, toWriteValue)
    readValue = mrf.read_long(address)
    if (printResults):
        print(str(name) + ": write value: " + str(toWriteValue) + ", read value: " + str(readValue))
    return toWriteValue == readValue

def assertReadShort(name, address, shouldBeValue, printResults):
    readValue = mrf.read_short(address)
    if (printResults):
        print(str(name) + ": read value: " + str(readValue) + ", should be: " + str(shouldBeValue))
    return shouldBeValue == readValue

def assertReadLong(name, address, shouldBeValue, printResults):
    readValue = mrf.read_long(address)
    if (printResults):
        print(str(name) + ": read value: " + str(readValue) + ", should be: " + str(shouldBeValue))
    return shouldBeValue == readValue

############ HANDLERS ############
def interrupt_routine():
    # print("Ocurrio una interrupcion")
    mrf.interrupt_handler()

def handle_rx():
    # print("received a packet {} bytes long\n".format(mrf.get_rxinfo().frame_length))
    if (mrf.get_bufferPHY()):
        print("Packet data (PHY Payload):\n")
        for i in range(0,mrf.get_rxinfo().frame_length):
            print(mrf.get_rxbuf()[i])

    # print("\r\nASCII data (relevant data):")
    # p_aux = mrf.get_rxinfo().panid
    s_aux = mrf.get_rxinfo().srcaddr
    # d_aux = mrf.get_rxinfo().destaddr

    msg_type = mrf.get_rxinfo().rx_data[0]
    if (msg_type == 0):     # ie. tipo sensor -> data
        sens_id = mrf.get_rxinfo().rx_data[1]
        data_type = mrf.get_rxinfo().rx_data[2]
        valor = (mrf.get_rxinfo().rx_data[3] << 8) | mrf.get_rxinfo().rx_data[4]
        m_aux = sepyrebase.Mensaje(s_aux,sens_id,data_type,valor)
        data_queue.put(m_aux)

    # for i in range(0,mrf.rx_datalength()):
    #     print(str(mrf.get_rxinfo().rx_data[i]) + "-",end="")

    # print("\r\nLQI/RSSI=", end="")
    # print(mrf.get_rxinfo().lqi, end="")
    # print("/", end="")
    # print(mrf.get_rxinfo().rssi)

def handle_tx():
    # print("handle_tx()" + str(mrf.get_txinfo().tx_ok))
    if (mrf.get_txinfo().tx_ok):
        # print("TX went ok, got ack")
        pass
    else:
        print("TX failed after {} retries\n".format(mrf.get_txinfo().retries))

def check_queue_data():
    global end_td
    while not end_td:
        if not (data_queue.empty()):
            m_data = data_queue.get()
            m_data.send_sens_val()

def check_queue_action():
    global end_ta
    while not end_ta:
        if not (action_queue.empty()):
            m_data = action_queue.get()
            cantIntentos = 0
            mrf.get_txinfo().tx_ok = 0
            while (mrf.get_txinfo().tx_ok != 1 and cantIntentos < 10):
                init_time_qa = int(round(time.time() * 1000))
                if (m_data[0] == ADDR_NODO_1):
                    # if (m_data[2]):
                    #     mrf.send_command(ADDR_NODO_1, m_data[1], 1)
                    # else:
                    #     mrf.send_command(ADDR_NODO_1, m_data[1], 0)
                    mrf.send_command(ADDR_NODO_1, m_data[1], m_data[2])
                elif (m_data[0] == ADDR_NODO_2):
                    # if (m_data[2]):
                    #     mrf.send_command(ADDR_NODO_2, m_data[1], 1)
                    # else:
                    #     mrf.send_command(ADDR_NODO_2, m_data[1], 0)
                    mrf.send_command(ADDR_NODO_2, m_data[1], m_data[2])
                elif (m_data[0] == ADDR_NODO_3):
                    # if (m_data[2]):
                    #     mrf.send_command(ADDR_NODO_3, m_data[1], 1)
                    # else:
                    #     mrf.send_command(ADDR_NODO_3, m_data[1], 0)
                    mrf.send_command(ADDR_NODO_3, m_data[1], m_data[2])
                init_time_qa = int(round(time.time() * 1000)) - init_time_qa
                print("Comando enviado en: "+ str(init_time_qa))
                cantIntentos += 1
                time.sleep(0.1)

############ Stream DB Functions ############
# NodoId - ActId - Accion
init_f11 = 1
init_f12 = 1
init_f13 = 1
init_f14 = 1
def f11(mensaje):
    global init_f11
    if init_f11: # A simple flag to discard the first call.
        init_f11 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_1,1,data]
        action_queue.put(arr)

def f12(mensaje):
    global init_f12
    if init_f12:
        init_f12 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_1,2,data]
        action_queue.put(arr)

def f13(mensaje):
    global init_f13
    if init_f13:
        init_f13 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_1,3,data]
        action_queue.put(arr)

def f14(mensaje):
    global init_f14
    if init_f14:
        init_f14 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_1,4,data]
        action_queue.put(arr)

init_f21 = 1
init_f22 = 1
init_f23 = 1
init_f24 = 1
def f21(mensaje):
    global init_f21
    if init_f21:
        init_f21 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_2,1,data]
        action_queue.put(arr)

def f22(mensaje):
    global init_f22
    if init_f22:
        init_f22 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_2,2,data]
        action_queue.put(arr)

def f23(mensaje):
    global init_f23
    if init_f23:
        init_f23 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_2,3,data]
        action_queue.put(arr)

def f24(mensaje):
    global init_f24
    if init_f24:
        init_f24 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_2,4,data]
        action_queue.put(arr)

init_f31 = 1
init_f32 = 1
init_f33 = 1
init_f34 = 1
def f31(mensaje):
    global init_f31
    if init_f31:
        init_f31 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_3,1,data]
        action_queue.put(arr)

def f32(mensaje):
    global init_f32
    if init_f32:
        init_f32 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_3,2,data]
        action_queue.put(arr)

def f33(mensaje):
    global init_f33
    if init_f33:
        init_f33 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_3,3,data]
        action_queue.put(arr)

def f34(mensaje):
    global init_f34
    if init_f34:
        init_f34 = 0
    else:
        data = mensaje["data"]
        arr = [ADDR_NODO_3,4,data]
        action_queue.put(arr)

################## MAIN ##################
last_time = 0
tx_interval = 1000
data_queue = queue.Queue()
action_queue = queue.Queue()

############ Stream DB ############
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),sepyrebase.ACT1,f11)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),sepyrebase.ACT2,f12)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),sepyrebase.ACT3,f13)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_1)),sepyrebase.ACT4,f14)

sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_2)),sepyrebase.ACT1,f21)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_2)),sepyrebase.ACT2,f22)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_2)),sepyrebase.ACT3,f23)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_2)),sepyrebase.ACT4,f24)

sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_3)),sepyrebase.ACT1,f31)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_3)),sepyrebase.ACT2,f32)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_3)),sepyrebase.ACT3,f33)
sepyrebase.set_actuador_callback(str(hex(ADDR_NODO_3)),sepyrebase.ACT4,f34)

if (run_test):
    print("\nRealizando test de lectura y escritura...")
    count = 2
    testDuration = int(round(time.time() * 1000))
    for i in range(0,count):
        mrf.reset()
        testRW(1)

    testDuration = int(round(time.time() * 1000)) - testDuration

    print("\n---------------Resumen Total---------------\n")
    print("Lecturas exitosas: " + str(totalReadSucc))
    print("Lecturas fallidas: " + str(totalRead - totalReadSucc))
    print("")
    print("Escrituras exitosas: " + str(totalWriteSucc))
    print("Escrituras fallidas: " + str(totalWrite - totalWriteSucc))
    print("")
    print("Test duration: " + str(testDuration) + " ms")
    print("")
    print("-------------------------------------------")

mrf.reset()
mrf.init()
mrf.set_channel(18)
mrf.set_pan(PANID)
mrf.address16_write(ADDR_NODO_1)
mrf.set_promiscuous(False)
mrf.set_palna(True)
mrf.set_interrupts()
# mrf.set_bufferPHY(true)

############ Attach Interruption Handler ############
wiringpi.wiringPiISR(pin_interrupt, wiringpi.GPIO.INT_EDGE_FALLING, interrupt_routine)

print("PANId: 0x{:04X} - Addr: 0x{:04X}".format(mrf.get_pan(),mrf.address16_read()))

last_time = int(round(time.time() * 1000))

### Thread que envia mensajes con datos de sensores a la bd.
end_td = 0
td = threading.Thread(target=check_queue_data,args="")
td.setDaemon(False)
td.start()

### Thread que envía las acciones a los actuadores.
end_ta = 0
ta = threading.Thread(target=check_queue_action,args="")
ta.setDaemon(False)
ta.start()

############ LOOP ############

while True:
    try:
        # current_time = int(round(time.time() * 1000))
        # if (current_time - last_time > tx_interval):
        #     last_time = current_time
            # print("Comunicando...")
            # mrf.send16(0x6005, "test comunicacion")
            # mrf.send_command(0x6003, 1, 1)
            # time.sleep(1)

        mrf.check_flags(handle_rx, handle_tx)
    except KeyboardInterrupt:
        break

print("\nSaliendo...")

sepyrebase.close_db()

mrf.close()

end_td = 1
end_ta = 1

td.join()
ta.join()

print("Finalizado.")

############ END MAIN ############




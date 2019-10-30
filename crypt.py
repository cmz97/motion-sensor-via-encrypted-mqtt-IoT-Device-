import esp32
import random
import os
import ubinascii
import machine
import hmac, hashlib
import json
import struct
from ucryptolib import aes
import utime


class CryptAes:
    """Uses AES encryption to encrypt Payload/Data before sending using MQTT protocol
    The AES algo uses Cipher Block Chaining (CBC)
    AES encryption correctly requires three things to produce ciphertext:
    a message: Payload/Data which is to be encrypted,
    a key: Piece of information (a parameter) that determines the functional output of a
        cryptographic algorithm. For encryption algorithms, a key specifies transformation
        of plaintext into ciphertext, and vice versa for decryption algorithms.
    initialization vector (IV): Piece of data sent along with the key that modifies the end
        ciphertext result. As the name suggests, it initializes the state of the encryption
        algorithm before the data enters. This protects against attacks like pattern analysis.
        This needs to be DIFFERENT for every message.

    Uses sessionID (received from Spinner #2) and Encrypted Data to generate HMAC for
    authentication of the sending device (Spinner #1)
    SessionID is generated by Spinner #2 by posting to the MQTT topic, SessionID every 1 sec.

    HMAC (hash-based message authentication code): Type of message authentication code (MAC)
    involving a cryptographic HASH function and a secret cryptographic key. It may be used to
    simultaneously verify both the data integrity and the authentication of a message.

    """
    #-----------------------------------------------COMMON-----------------------------------------------------------#


    def __init__(self, sessionID):
        """
        This class initializes all keys and ids
        nodeid     : unique id to identify device or board
        iv         : pseudorandom initialization vector, this needs to be DIFFERENT for every message.
        staticiv   : static initialization vector to obfuscate the randomized
                     initialization vector sent with each message, NOT used for any data
        ivkey      : unique key to encrypt the initialization vector
        datakey    : unique key to encypt the Payload/Data
        passphrase : key to generate the HMAC code for authentication

        sessionID  : unique value to identify the current communication session, generated only by Spinner #2

        ***********************NOTE******************************
        AES is a block cipher with a block size of 128 bits; that's why it encrypts 16 bytes at a time.
        The block size of CBC mode of encryption is 16, make sure that any data going into AES
        Encryption is of size 16 bytes.
        """

        self.nodeid = 'jR4Zb7JqF8TUUoDs' #Spinner 1
        self.iv = ''.join([chr(random.randint(65,90)) for i in range(16)])
        self.staticiv = 'yfV6fFvbzdauzCgJ'

        self.ivkey = 'YuHwgB9ehcjTqFvQ'
        self.datakey = 'b3RSDbYt4TfkoU5y'
        self.passphrase = 'KQrFKpn85e4ekmVp'

        self.sessionID = sessionID

    #------------------------------------SPINNER #1 Needs to Use These Functions--------------------------------------#


    def blockEncrypterAES(self, encryptorInstance, hexString):
        div, rem = divmod(len(hexString),16)
        returnStr = bytes()
        hexString = str(hexString).strip('b\'').strip('\'')
        for i in range(div):
            returnStr += encryptorInstance.encrypt(bytes(hexString[i*16: i*16 + 16], 'utf-8'))
        if rem != 0:
            padding = 'X'* (16 - rem)
            print('hexString[div*16:]+padding:'+hexString[-rem:]+padding)
            returnStr += encryptorInstance.encrypt(bytes(hexString[div*16:]+padding,'utf-8'))
        return returnStr

    def encrypt(self, sensor_data):
        """Encrypt each of the current initialization vector (iv), the nodeid, and the sensor_data
        :param sensor_data  : Acceleration X, Acceleration Y, Acceleration Z, and Temperature
        """
        myAES = aes(self.ivkey,2,self.staticiv)# using (staticiv, ivkey) for iv
        print(self.iv)
        print(bytes(self.iv,'utf-8'))
        self.encrypted_iv = myAES.encrypt(bytes(self.iv,'utf-8'))
        print(self.encrypted_iv)

        myAES = aes(self.datakey,2,self.iv) #(iv, datakey) for nodeid and sensor_data
        self.encrypted_nodeid = myAES.encrypt(bytes(self.nodeid,'utf-8'))
        self.encrypted_sensor_data = self.blockEncrypterAES(myAES, sensor_data)


    def sign_hmac(self, sessionID):
        """Generate HMAC by using passphrase, and combination of encrypted iv, encrypted nodeid,
        encrypted data, received sessionID.
        :param sessionID: unique value to identify the current communication session
        :return         : generated HMAC
        """
        dataPreProcess = self.encrypted_iv + self.encrypted_nodeid + self.encrypted_sensor_data + sessionID
        myHMAC = hmac.HMAC(bytes(self.passphrase,'utf-8'),msg = dataPreProcess, digestmod = hashlib.sha224)
        return myHMAC.hexdigest()

    def send_mqtt(self, hmac_signed):
        """Prepare the message for MQTT transfer using all of encrypted iv, encrypted nodeid,
        encrypted data, HMAC. Create the message in JSON format.
        :param hmac_signed  : generated HMAC
        :return             : MQTT message to publish to Spinner #2 on Topic "Sensor_Data"
        """
        data = {}
        data['e_iv'] = ubinascii.hexlify(self.encrypted_iv)
        data['e_nodeid'] =  ubinascii.hexlify(self.encrypted_nodeid)
        data['e_sd'] =  ubinascii.hexlify(self.encrypted_sensor_data)
        data['HMAC'] =  hmac_signed
        print('PROCESSED JSON: '+ str(data))
        return json.dumps(data)


    #------------------------------------SPINNER #2 Needs to Use These Functions--------------------------------------#


    def verify_hmac(self, payload):
        """Authenticates the received MQTT message.
        Generate HMAC using passphrase, sessionID, RECEIVED encrypted iv, encrypted nodeid, encrypted data
        and compare with received hmac inside payload to authenticate.
        :param payload  : received MQTT message from Spinner #1. This includes all encrypted data, nodeid, iv, and HMAC
        :return message : MQTT message to publish to Spinner #1 on Topic "Acknowledge", can be "Failed Authentication"
                          if verification is unsuccessful
        """


    def decrypt(self, payload):
        """Decrypts the each encrypted item of the payload.
        Initialize decryption cipher for each item and and use cipher to decrypt payload items.
        :param payload  : received MQTT message from Spinner #1. This includes all encrypted data, nodeid, iv, and HMAC
        :return         : MQTT message to publish to Spinner #1 on Topic "Acknowledge", can be "Successful Decryption"
        """

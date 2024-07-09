#include "AES.h"
#include "base64.h"

AES aes;

uint8_t getrnd() {
    uint8_t really_random = *(volatile uint8_t *)0x3FF20E44;
    Serial.println(really_random);
    return really_random;
}

// Generate a random initialization vector
void gen_iv(byte  *iv) {
    for (int i = 0 ; i < N_BLOCK ; i++ ) {
        iv[i]= (byte) getrnd();
    }
}
    
void setup() {
    Serial.begin(115200);
//    while (!Serial) ;
    
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("\nBooting...");  

    char b64data[200];
    byte cipher[1000];
    byte iv [N_BLOCK] ;
    
    Serial.println("Let's encrypt:");

    byte key[] = {0x4d,0x79,0x53,0x65,0x63,0x72,0x65,0x74,0x4b,0x65,0x79,0x65,0x73,0x77,0x33,0x39};

    byte my_iv[N_BLOCK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    String msg = "TrYing to encrypt message"; // String to encrypt
    
    aes.set_key( key , sizeof(key));  // Get the globally defined key
    gen_iv( my_iv );                  // Generate a random IV
    Serial.println((char*)my_iv );
    // Print the IV
    base64_encode( b64data, (char *)my_iv, N_BLOCK);
    Serial.println(" IV b64: " + String(b64data));

    Serial.println(" Message: " + msg );
 
    int b64len = base64_encode(b64data, (char *)msg.c_str(),msg.length());
    Serial.println (" Message in B64: " + String(b64data) );
    Serial.println (" The lenght is:  " + String(b64len) );
    
    // For sanity check purpose
    //base64_decode( decoded , b64data , b64len );
    //Serial.println("Decoded: " + String(decoded));
    
    // Encrypt! With AES128, our key and IV, CBC and pkcs7 padding    
    aes.do_aes_encrypt((byte *)b64data, b64len , cipher, key, 128, my_iv);
    
    Serial.println("Encryption done!");
    
    Serial.println("Cipher size: " + String(aes.get_size()));
    
    base64_encode(b64data, (char *)cipher, aes.get_size() );
    Serial.println ("Encrypted data in base64: " + String(b64data) ); // Data we need to send to thingspeak
      
    Serial.println("Done...");
    delay(10000);
}
//JS code

//String.prototype.hexEncode = function(){
//    var hex, i;
//
//    var result = "";
//    for (i=0; i<this.length; i++) {
//        hex = this.charCodeAt(i).toString(16);
//        result += (hex).slice(-4);
//    }
//
//    return result
//};
// var AESKey = 'MySecretKeyesw39'.hexEncode();
// var esp8266_iv  = 'AAAAAAAAAAAAAAAAAAAAAA=='; // copy
// var esp8266_msg = 'PgZ/Te2Pvshyb1ylYvInvu0f/3n9yhDmKwVfYWb9mpVzJ6oHJsVAZnCoNBmqqXrM'; // copy
// var iv = CryptoJS.enc.Utf8.parse(atob(esp8266_iv));
// var key= CryptoJS.enc.Hex.parse( AESKey );
// var bytes  = CryptoJS.AES.decrypt( esp8266_msg, key , { iv: iv,mode:CryptoJS.mode.CBC} );
// function trim (s, c) {
//  if (c === "]") c = "\\]";
//  if (c === "^") c = "\\^";
//  if (c === "\\") c = "\\\\";
//  return s.replace(new RegExp(
//    "^[" + c + "]+|[" + c + "]+$", "g"
//  ), "");
//}
//atob(trim(bytes.toString(CryptoJS.enc.Utf8),"=")) 

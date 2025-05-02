#include "ota.h"
#define TINY_GSM_MODEM_SIM800
const char apn[] = "internet";
#include <TinyGsmClient.h>
TinyGsm        modem(Serial2);
#include <ArduinoHttpClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//explore the possibility of reading data from flash storage uisng the progmem feature
//for testing uploads to firebase storage.
//const char FirebaseDatabaseURL[]="smart-water-quality-moni-8953e-default-rtdb.europe-west1.firebasedatabase.app";
const char ThingspeakServer[]="api.thingspeak.com";
const char mqttServer[]= "mqtt3.thingspeak.com";
String api_key="FUFK5ET7A8C840X7";
//these parameters should be configurable too, hopefully read from eeprom, or from the sd card on startup
//eeprom however stores strings in lower case !! this will affect the 
const char clientID[]="MAENMAU1NRITDxAdPCU8FCM";
const char mqttUserName[]="MAENMAU1NRITDxAdPCU8FCM";
const char mqttPass[]="+sbgJyvYU2W1t0NtBGSeBH1i";
//int fieldsToPublish[8]={1,1,1,1,1,1,1,1};
//float dataToPublish[8]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00}; //array of data to publish to fields
char controlData[400]="";
int year,month,day,hour,min,sec ;
float timezone;
int new_entry_id=0,old_entry_id=0;
bool gsmPowerStatus=true;
bool updateAvailable=false;
bool updateEligibility=false;
char endOfHeaders[]="\r\n\r\n";
String checkBalances()
{
    //simcard number
    if(modem.getOperator()=="MTN-UGANDA")
    {
        Serial.println(modem.sendUSSD("*135#"));
        return modem.sendUSSD("*131#");
    }
    else if(modem.getOperator()=="CelTel Cellular")
    {
        Serial.println (modem.sendUSSD("*100*7*2#"));
        return modem.sendUSSD("*175*4#");
    }
    else
        return modem.sendUSSD("*131#");
    // if(balance.indexOf("MB")>0||balance.indexOf("GB")>0)
    // {
    //     // Serial.println(balance.indexOf("MB"));
    //     balance=balance.substring(balance.indexOf("MB")-3);
    // }
    // return balance;
}
String getGsmTimeStamp()
{
    String timeStamp;
    if (modem.getNetworkTime(&year, &month, &day, &hour, &min, &sec, &timezone))
         {
             timeStamp=String(year)+"-"+String(month)+"-"+String(day)+"T"+String(hour)+":"+String(min)+":"+String(sec)+"-"+String(timezone);
         }
    return timeStamp;
}
bool turnOffGsm(uint8_t gsmSwitch)
{
    if(modem.isGprsConnected())
    {
        modem.gprsDisconnect();
        //modem.poweroff();
    }
    modem.radioOff();
    gsmPowerStatus=false;
    return gsmPowerStatus;
}
void startGsm(uint8_t gsmSwitch=-1)
{
    Serial.println(F("starting gsm now"));
    Serial2.begin(9600);
    if(!modem.isNetworkConnected())
    {
        modem.init();
        modem.waitForNetwork();
    }
    Serial.println(modem.getModemInfo());
    if(modem.isNetworkConnected())
    {
        Serial.println(getGsmTimeStamp());
        Serial.print(F("connected to:"));
        Serial.println(modem.getOperator());
        getGsmTimeStamp();
        Serial.println(modem.getSignalQuality());
        Serial.println(checkBalances());
        if(!modem.isGprsConnected())
        {
            modem.gprsConnect(apn);
        }
       
        if(modem.isGprsConnected())
        {
            Serial.print(F("connected to:"));Serial.println(apn);
        }
    }
}
bool checkForUpdate(String channel)
{
    //Serial.println(ThingspeakServer);
    if(!modem.isGprsConnected())
    {
        modem.gprsConnect(apn);
    }
    if(modem.isGprsConnected())
    {
        Serial.println(modem.getLocalIP());
        TinyGsmClient client(modem);
        HttpClient http(client,ThingspeakServer,80);
        Serial.println(F("checking for update token from thingspeak"));
        String contentType = "application/json";
        http.connect(ThingspeakServer,80);
        http.connectionKeepAlive();
        int err=http.get("/channels/"+channel+"/feeds/last.json?api_key=" + api_key + "&results=1");
        if(err==0&&http.responseStatusCode()==200)
        {
            //Serial.println(http.responseStatusCode());
            if(!http.find(endOfHeaders))
            {
                Serial.println(F("invalid resonse"));
            }
            if(http.isResponseChunked())
            {
                Serial.println(F("response is chuncked"));
            }
            unsigned long timeoutStart=millis();
            StaticJsonDocument<400> doc;
            while(http.connected()&&http.available()&&!http.endOfBodyReached()&&millis()-timeoutStart<30000L)
            {
                 http.connectionKeepAlive();
                if(http.available())
                {
                    http.readBytesUntil('\r',controlData,sizeof(controlData));
                }
                timeoutStart=millis();
            }
            controlData[1]='{';
            DeserializationError error=deserializeJson(doc ,controlData);
            serializeJsonPretty(doc,Serial);
            if(error){
                Serial.print(F("deserializeJson() failed:"));
                Serial.println(error.f_str());
                return;
            }
            Serial.println();
            //capturing the control parameters to use to configure device remotely
            http.endRequest();
            http.stop();
        }
        else if(err!=0)
        {
            Serial.println(F("failed to connect to server"));
            http.endRequest();
            http.stop();
            modem.gprsDisconnect();
            delay(2000);
        }
        else if(http.responseStatusCode()!=200)
            {
                Serial.print(F("response status code:"));
                Serial.println(http.responseStatusCode());
                Serial.println(F("check your connection parameters please"));
                http.endRequest();
                http.stop();
                modem.gprsDisconnect();
            }
        client.stop();
        
    }
    //modem.gprsDisconnect();
    return updateAvailable; 
}
void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int mesLength ) {
    
    char p[mesLength + 1];
    memcpy( p, payload, mesLength );
    p[mesLength] = NULL;
    Serial.print( F("Answer: " ));
    Serial.println( String(p) );
}
void mqttUpdate(String channel,int data[])
{   
    TinyGsmClientSecure client(modem);
    PubSubClient  mqttClient(client);
    if(!modem.isNetworkConnected())
    {
        modem.waitForNetwork(20000L,1);
    }
    if(modem.isNetworkConnected())
    {
        Serial.println(F("network already connected"));
        if(!modem.isGprsConnected()){modem.gprsConnect(apn);}
        if(modem.isGprsConnected())
        {
            Serial.println(F("connected to internet"));
            mqttClient.setServer(mqttServer,8883);
            mqttClient.setCallback( mqttSubscriptionCallback );
            mqttClient.setBufferSize( 2048 );
            uint8_t count=0;
            //connect to mqtt client
            while ( !mqttClient.connected() )
            {
                // Connect to the MQTT broker.
                Serial.println(F("connecting to thingspeak mqtt broker"));
                if ( mqttClient.connect( clientID, mqttUserName, mqttPass ) ) {
                Serial.print( F("MQTT to ") );
                Serial.print( mqttServer );
                Serial.print (F(" at port "));
                Serial.print( 8883 );
                Serial.println( F(" successful.") );
                } else {
                    Serial.print( F("MQTT connection failed, rc = " ));
                    // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
                    Serial.print( mqttClient.state() );
                    Serial.println( F(" Will try again in a few seconds") );
                    delay( 100 );
                    count++;
                    }
                if(count>=3)
                    break;
            }
            if(mqttClient.connected())
            {
                
                uint8_t index=0;
                String dataString="";
                while (index<8){
                    //  build the posting string to send to ThingSpeak.
                    dataString+="&field" + String( index+1 ) + "="+String( data [ index ] );
                    index++;
                }
                if (index == 8) {
                    //dataString += "&field8=\"""";
                    dataString += "&field8=";
                    dataString+=data[7];
                    dataString += ",";
                    dataString +=data[8];
                    dataString += ",";
                    dataString +=data[9];
                    dataString += ",";
                    dataString +=data[10];
                    dataString += ",";
                    dataString +=data[11];
                    dataString += ",";
                    dataString +=data[12];
                    dataString += ",";
                    dataString +=data[13];
                    dataString += ",";
                    dataString +=data[14];
                    dataString += ",";
                    dataString +=data[15];
                    dataString += ",";
                    dataString +=data[16];
                    dataString += ",";
                    dataString +=data[17];
                    //dataString += "\"";
                }
                Serial.println( dataString );
                // Create a topic string and publish data to ThingSpeak channel feed.
                String topicString ="channels/" + channel + "/publish";
                mqttClient.publish( topicString.c_str(), dataString.c_str() );
                //Serial.println( "Published to channel " +channel);
                delay(1000);
            }
            //mqttClient.disconnect();
            //client.stop();
        }
        //modem.gprsDisconnect();
    }
    mqttClient.loop();
}

// /.json?auth
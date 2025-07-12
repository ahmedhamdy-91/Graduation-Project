#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// إعداد الاتصال مع وحدة GSM باستخدام SoftwareSerial
SoftwareSerial gsmSerial(7, 8);  // Pin 7 لـ RX و Pin 8 لـ TX للاتصال بوحدة GSM

// إعداد الاتصال مع مستشعر GPS باستخدام SoftwareSerial
SoftwareSerial gpsSerial(4, 5);  // Pin 4 لـ RX و Pin 5 لـ TX لمستشعر GPS

TinyGPSPlus gps;  // كائن لتخزين بيانات GPS

// أرقام الهواتف
String userPhoneNumber = "+201234567890";  // رقم المستخدم
String adminPhoneNumber = "+2010987654321";  // رقم المسؤول

// الحد الأدنى للرصيد (مثلاً 10 وحدات أو مبلغ معين)
int lowBalanceThreshold = 10;

// تعريف الأزرار
const int callButtonPin = 2;  // Pin 2 لزر المكالمة
const int smsButtonPin = 3;   // Pin 3 لزر إرسال الرسالة
const int led1 = 11;
const int led2 = 12;


void setup() {
  // إعداد الاتصال التسلسلي لمراقبة البيانات عبر الكمبيوتر
  Serial.begin(9600);
  gsmSerial.begin(9600);  // إعداد الاتصال مع وحدة GSM
  gpsSerial.begin(9600);  // إعداد الاتصال مع وحدة GPS

  // إعداد الأزرار كمدخلات مع تفعيل مقاومة السحب الداخلية
  pinMode(callButtonPin, INPUT_PULLUP);  // زر المكالمة
  pinMode(smsButtonPin, INPUT_PULLUP);   // زر إرسال الرسالة
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);


  // إرسال رسالة تنبيهية عند بداية تشغيل النظام
 // sendSMSHelper(userPhoneNumber, "System started, awaiting GPS data...");
}

void loop() {

  // التحقق من وجود رسائل SMS واردة
checkForSMS();

// التحقق من رصيد الشريحة
checkBalance();

  // قراءة بيانات GPS بشكل مستمر
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      float latitude = gps.location.lat();  // خط العرض
      float longitude = gps.location.lng(); // خط الطول

      // طباعة الإحداثيات في شاشة Serial (اختياري)
      Serial.print("Latitude= "); 
      Serial.print(latitude, 6); 
      Serial.print(" Longitude= "); 
      Serial.println(longitude, 6);
    }
  }


  // فحص حالة الأزرار:
  if (digitalRead(callButtonPin) == HIGH) {  // إذا تم ضغط زر المكالمة
    makeCall(adminPhoneNumber);  // إجراء المكالمة للمسؤول
  }

  if (digitalRead(smsButtonPin) == HIGH) {  // إذا تم ضغط زر إرسال الرسالة
    sendSMS(adminPhoneNumber);  // إرسال الموقع إلى المسؤول
  }
}

// دالة لإجراء المكالمة
void makeCall(String recipient) {
  gsmSerial.println("ATD" + recipient + ";");  // إجراء مكالمة إلى الرقم المحدد
  digitalWrite(led2, HIGH);
  delay(5000);  // الانتظار لمدة 5 ثوانٍ (حسب الوقت الذي تحتاجه)
  gsmSerial.println("ATH");  // إنهاء المكالمة
  digitalWrite(led2, LOW);
  
}

// دالة لإرسال رسالة SMS مع الموقع
void sendSMS(String recipient) {
  float latitude = gps.location.lat();  // خط العرض
  float longitude = gps.location.lng(); // خط الطول

  String message = "This is an emergency message. My current location is: ";
  message += String(latitude, 6) + ", " + String(longitude, 6);  // إضافة الإحداثيات إلى الرسالة

  // إرسال الرسالة النصية إلى الرقم المحدد
  sendSMSHelper(recipient, message);
}

// دالة لإرسال رسالة SMS
void sendSMSHelper(String recipient, String message) {
  gsmSerial.println("AT+CMGF=1");  // وضع وحدة GSM في وضع SMS
  delay(100);
  gsmSerial.println("AT+CMGS=\"" + recipient + "\"");  // الرقم الذي سيتم إرسال الرسالة إليه
  delay(100);
  gsmSerial.println(message);  // نص الرسالة
  delay(100);
  gsmSerial.println((char)26);  // إنهاء الرسالة (Ctrl+Z)
  delay(100);
  digitalWrite(led1, HIGH);
}


// دالة للتحقق من وجود رسالة SMS
void checkForSMS() {
  gsmSerial.println("AT+CMGL=\"ALL\"");  // قراءة جميع الرسائل
  delay(1000);  // الانتظار لوقت قليل

  while (gsmSerial.available()) {
    String message = gsmSerial.readString();
    Serial.println(message);  // طباعة الرسالة الواردة على Serial

    // استخراج الرقم المُرسل
    int senderIndex = message.indexOf("+20");
    String senderNumber = message.substring(senderIndex, senderIndex + 13);

    // إذا كانت الرسالة تحتوي على كلمة "موقع الكرسي"
    if (message.indexOf("Wheelchair Location") != -1) {
      sendLocationSMS(senderNumber);  // إرسال الموقع إلى الرقم المُرسل
    }

    // مسح الرسالة بعد معالجتها
    deleteSMS();
  }
}


// دالة لإرسال رسالة SMS مع الموقع
void sendLocationSMS(String recipient) {
  float latitude = gps.location.lat();  // خط العرض
  float longitude = gps.location.lng(); // خط الطول

  String message = "Wheelchair Location: ";
  message += String(latitude, 6) + ", " + String(longitude, 6);  // إضافة الإحداثيات إلى الرسالة

  // إرسال الرسالة النصية إلى الرقم المحدد
  sendSMSHelper(recipient, message);
}


// دالة للتحقق من رصيد الشريحة
void checkBalance() {
  gsmSerial.println("AT+CSQ");  // التحقق من قوة الإشارة أو الرصيد
  delay(1000);
  
  while (gsmSerial.available()) {
    String response = gsmSerial.readString();
    Serial.println(response);  // طباعة الاستجابة للحصول على تفاصيل

    if (response.indexOf("+CSQ") != -1) {
      int signalStrength = response.substring(response.indexOf(":") + 1).toInt();
      
      if (signalStrength < lowBalanceThreshold) {
        // إرسال تنبيه عن انخفاض الرصيد إلى كلا الرقمين
        sendSMSHelper(userPhoneNumber, "Warning: The balance is low, please charge the SIM card");
        sendSMSHelper(adminPhoneNumber, "Warning: The balance is low, please charge the SIM card");
      }
    }
  }
}

// دالة لمسح الرسائل القديمة بعد معالجتها
void deleteSMS() {
  gsmSerial.println("AT+CMGD=1");  // مسح الرسالة الأولى في القائمة (يمكن تغيير الرقم حسب الفهرس)
  delay(1000);  // الانتظار لحين تنفيذ الأمر
}
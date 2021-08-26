#pragma once

#include <TinyGPS++.h>

#define GPS_RX_PIN 2
#define GPS_DEFAULT_BAUD 9600

class GPS {
  public:
    GPS();

    void setGoal(double lat, double lng);
    void encode();
    void getHeader(String& buffer);
    void readValues(String& buffer);

    double hdop = 0.0;
    bool locationIsValid = false;
    double distanceToGoal = 0.0;
    double courseToGoal = 0.0;
    double speedKmph = 0.0;
    double course = 0.0;

    TinyGPSPlus gps;

  private:
    HardwareSerial *ss;
    double goalLat = 0.0;
    double goalLong = 0.0;
};

GPS::GPS() {
  ss = new HardwareSerial(GPS_RX_PIN);
  ss->begin(GPS_DEFAULT_BAUD);
  ss->write("$PMTK251,115200*1F\r\n");
  ss->updateBaudRate(115200);
  ss->flush();
  ss->write("$PMTK300,100,0,0,0,0*2C\r\n");
  ss->write("$PMTK220,100*2F\r\n");
  ss->write("$PMTK314,10,1,10,1,10,10,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  ss->flush();
}

void GPS::setGoal(double lat, double lng) {
  goalLat = lat;
  goalLong = lng;
}

void GPS::encode() {
  while (ss->available()) {
    gps.encode(ss->read());
  }
}

void GPS::getHeader(String& buffer) {
  buffer += String("GPS,Testing TinyGPS++ library v. ");
  buffer += String(TinyGPSPlus::libraryVersion());
  buffer += String("\n");
  buffer += String("GPS,Sats,HDOP,Latitude,Longitude,Fix Age,DateTime,DateAge,Alt,Course,Speed,Card,DistanceToG,CourseToG,CardToG,CharsRX,SentencesRX,ChecksumFail\n");
}

void GPS::readValues(String& buffer) {
  buffer += "GPS,";

  if (gps.satellites.isValid()) {
    buffer += String(gps.satellites.value());
  }
  buffer += String(",");

  if (gps.hdop.isValid()) {
    hdop = gps.hdop.hdop();
    buffer += String(hdop, 6);
  }
  buffer += String(",");

  if (gps.location.isValid()) {
    buffer += String(gps.location.lat(), 6);
  }
  buffer += String(",");

  if (gps.location.isValid()) {
    buffer += String(gps.location.lng(), 6);
  }
  buffer += String(",");

  if (gps.location.isValid()) {
    buffer += String(gps.location.age());
  }
  buffer += String(",");

  if (gps.date.isValid())
  {
    buffer += String(gps.date.year());
    buffer += String("-");
    uint8_t month = gps.date.month();
    if (month < 10) {
      buffer += String("0");
    }
    buffer += String(month);
    buffer += String("-");
    uint8_t day = gps.date.day();
    if (day < 10) {
      buffer += String("0");
    }
    buffer += String(day);
    buffer += String("T");
  }
  if (gps.time.isValid())
  {
    uint8_t hour = gps.time.hour();
    if (hour < 10) {
      buffer += String("0");
    }
    buffer += String(hour);
    buffer += String(":");
    uint8_t minute = gps.time.minute();
    if (minute < 10) {
      buffer += String("0");
    }
    buffer += String(minute);
    buffer += String(":");
    uint8_t second = gps.time.second();
    if (second < 10) {
      buffer += String("0");
    }
    buffer += String(second);
  }
  buffer += String(",");

  if (gps.date.isValid())
  {
    buffer += String(gps.date.age());
  }
  buffer += String(",");


  if (gps.altitude.isValid()) {
    buffer += String(gps.altitude.meters(), 6);
  }
  buffer += String(",");

  if (gps.course.isValid()) {
    course = gps.course.deg();
    buffer += String(course, 6);
  }
  buffer += String(",");

  if (gps.speed.isValid()) {
    speedKmph = gps.speed.kmph();
    buffer += String(speedKmph, 6);
  }
  buffer += String(",");

  if (gps.course.isValid()) {
    buffer += String(TinyGPSPlus::cardinal(gps.course.deg()));
  }
  buffer += String(",");

  distanceToGoal =
    TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      goalLat,
      goalLong);

  if (gps.location.isValid()) {
    buffer += String(distanceToGoal, 6);
  }
  buffer += String(",");

  courseToGoal =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      goalLat,
      goalLong);

  if (gps.location.isValid()) {
    buffer += String(courseToGoal, 6);
  }
  buffer += String(",");

  const char *cardinalToGoal = TinyGPSPlus::cardinal(courseToGoal);

  if (gps.location.isValid()) {
    buffer += String(cardinalToGoal);
  }
  buffer += String(",");

  buffer += String(gps.charsProcessed());
  buffer += String(",");
  buffer += String(gps.sentencesWithFix());
  buffer += String(",");
  buffer += String(gps.failedChecksum());
  buffer += String("\n");
}


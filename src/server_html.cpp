#include "server_html.h"

String SendHTML_onCalibration(float xA,float yA,float zA, float xG,float yG,float zG){
  String ptr = "<html><head><!DOCTYPE html><meta http-equiv='Refresh' content='3' /><meta charset=utf-8></head><body>";
  ptr+="xA=  ";
  ptr+=xA;
  ptr+=" &deg; yA=  ";
  ptr+=yA;
  ptr+=" &deg; zA= ";
  ptr+=zA;
  ptr+="xG=  ";
  ptr+=xG;
  ptr+=" &deg; yG=  ";
  ptr+=yG;
  ptr+=" &deg; zG= ";
  ptr+=zG;
  ptr+="</body></html>";
  return ptr;
}

#ifndef EXPERIMENTAL
String SendHTML(float x,float y,float z, float dX, float dY, float dZ, int nomer_izmerenia, int timer){
  String ptr ="";
  //ptr+=IP;
  ptr+="  ";
  ptr+=x;
  ptr+="  ";
  ptr+=y;
  ptr+="  ";
  ptr+=z;
  ptr+="  ";
  ptr+=nomer_izmerenia;
  ptr+="  ";
  ptr+=timer;
  ptr+="";
  return ptr;
}
#endif

#ifdef EXPERIMENTAL
String SendHTML(float x,float y,float z, float dX, float dY, float dZ, int nomer_izmerenia){
  String ptr = "<html><head><!DOCTYPE html><meta http-equiv='Refresh' content='1' /><meta charset=utf-8></head><body>";
  ptr+="x=  ";
  ptr+=x;
  ptr+="  ";
  ptr+=" &deg; y=  ";
  ptr+=y;
  ptr+="  ";
  ptr+=" &deg; z= ";
  ptr+=z;
   ptr+="  ";
//  //ptr+="dx=  ";
//   ptr+=dX;
//   ptr+="  ";
//   //ptr+=" &deg; dy=  ";
//   ptr+=dY;
//   ptr+="  ";
//   //ptr+=" &deg; dz= ";
//   ptr+=dZ;

  //ptr+="  ";
  ptr+=" &deg; <br> Номер измерения: ";
  ptr+=nomer_izmerenia;
  ptr+=";";
  ptr+="</body></html>";
  return ptr;
}
#endif


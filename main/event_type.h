/*  Relais control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef RELAIS_H
#define RELAIS_H


typedef struct
{
   uint8_t out_event; //indicates that an event has occured
   uint8_t out_num; //which relay is choosen?
}out_event_type;


//not sure...
typedef struct
{
   uint8_t vol_events; //how many volume entrys?
   uint8_t volume; //the volume itself as a field of chars max value 24 steps
}vol_event_type;

out_event_type output;
vol_event_type vol;






#endif /* RELAIS_H */

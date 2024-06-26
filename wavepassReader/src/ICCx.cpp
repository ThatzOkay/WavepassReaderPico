#include "ICCx.h"
#include "Cipher.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#define ICCX_DEBUG
//#define LOCK_ONLY_ISO15693

Cipher crypto;

static bool iccx_key_exchange(uint8_t node_id)
{
    static uint8_t ard_key[4] = {0x29,0x23,0xbe,0x84};
    static uint8_t dev_key[4] = {0,0,0,0};
    struct ac_io_message msg;

    msg.addr = node_id;
    msg.cmd.code = ac_io_u16(AC_IO_CMD_ICCx_KEY_EXCHANGE);
    msg.cmd.nbytes = 4;
    memcpy(&msg.cmd.raw, ard_key, 4);

    if (!acio_send_and_recv(
            &msg, offsetof(struct ac_io_message, cmd.raw) + msg.cmd.nbytes)) {
        #ifdef ICCX_DEBUG
        printf("Starting queue loop failed");
            #endif
        return false;
    }    

    memcpy(dev_key, msg.cmd.raw, 4);
    
#ifdef ICCX_DEBUG
printf("Key exchange complete, got ");
for (int i=0; i<4; i++)
{
  printf(" ");
      if (dev_key[i] < 0x10) printf("0");
      printf("%X", dev_key[i]);
}
printf("\n");
            #endif

            unsigned long client_key = ((unsigned long) ard_key[0]) <<24 | ((unsigned long) ard_key[1]) <<16 | ((unsigned long) ard_key[2]) <<8 | (unsigned long) ard_key[3];
            unsigned long reader_key = ((unsigned long) dev_key[0]) <<24 | ((unsigned long) dev_key[1]) <<16 | ((unsigned long) dev_key[2]) <<8 | (unsigned long) dev_key[3];

            crypto.setKeys(client_key,reader_key);

    sleep_ms(200);
    return true;
}

static bool iccx_queue_loop_start(uint8_t node_id, bool encrypted)
{
    struct ac_io_message msg;

    msg.addr = node_id;
    msg.cmd.code = ac_io_u16(AC_IO_CMD_ICCx_QUEUE_LOOP_START);
    msg.cmd.nbytes = 1;
    msg.cmd.count = 0;

    if (!acio_send_and_recv(
            &msg, offsetof(struct ac_io_message, cmd.raw) + msg.cmd.nbytes)) {
        #ifdef ICCX_DEBUG
        printf("Starting queue loop failed");
            #endif
        return false;
    }    
    sleep_ms(200);
    return encrypted?iccx_key_exchange(node_id):true;
}

bool iccx_init(uint8_t node_id, bool encrypted)
{
    if (!iccx_queue_loop_start(node_id + 1, encrypted)) {
        return false;
    }

    return true;
}

static bool iccx_set_state(
    uint8_t node_id, int slot_state, icca_state_t *state)
{
    struct ac_io_message msg;

    msg.addr = node_id + 1;
    msg.cmd.code = ac_io_u16(AC_IO_CMD_ICCx_SET_SLOT_STATE);
    msg.cmd.nbytes = 2;
    /* buffer size of data we expect */
    msg.cmd.raw[0] = sizeof(icca_state_t);
    msg.cmd.raw[1] = slot_state;

    if (!acio_send_and_recv(
            &msg, offsetof(struct ac_io_message, cmd.raw) + msg.cmd.raw[0])) {
        #ifdef ICCX_DEBUG
        printf("Setting state of node ");
        printf("{0}", node_id + 1);
        printf(" failed \n");
            #endif
        return false;
    }

    if (state != NULL) {
        memcpy(state, msg.cmd.raw, sizeof(icca_state_t));
    }

    return true;
}

bool iccx_eject_card(icca_slot_state_t post_state)
{
  if (!iccx_set_state(
                0, AC_IO_ICCA_SLOT_STATE_EJECT, NULL)) {
            return false;
                }
       
  if ((post_state != 0) && !iccx_set_state(0, post_state, NULL)) 
            {
            return false;
            }
            return true;
}

static bool iccx_get_state(uint8_t node_id, iccx_state_t *state, bool encrypted)
{
    struct ac_io_message msg;
    static icca_state_t icca_state;

    if (encrypted) 
    {
      sleep_ms(60); // wait a little before requesting the state when in encrypted mode (else ICCB fails)
    }
    
    msg.addr = node_id + 1;
    msg.cmd.code = ac_io_u16(encrypted? AC_IO_CMD_ICCx_FEL_POLL : AC_IO_CMD_ICCx_POLL);
    msg.cmd.nbytes = 1;
    /* buffer size of data we expect */
    msg.cmd.count = sizeof(iccx_state_t);

    if (!acio_send_and_recv(
            &msg, offsetof(struct ac_io_message, cmd.raw) + msg.cmd.count)) {
                      #ifdef ICCX_DEBUG
        printf("Getting state of node ");
        printf("%d", node_id + 1);
        printf(" failed");
            #endif
        return false;
    }
    
    if (encrypted)
    /* got the encrypted data, decrypt it and check crc */
    {
      crypto.crypt(msg.cmd.raw,18);
      #ifdef ICCX_DEBUG
      printf("DECRYPTED : ");
      for (int i=0; i<18; i++)
      {
        if(msg.cmd.raw[i] < 0x10) printf("0");
        printf("%X", msg.cmd.raw[i]);
        printf(" ");
      }
      printf("\n");
      #endif

      /* last two bytes are the CRC */
      uint16_t crc = msg.cmd.raw[16] << 8 | msg.cmd.raw[17];
      uint16_t crc_calc = crypto.CRCCCITT(msg.cmd.raw, 16);
      if (crc != crc_calc) {
        #ifdef ICCX_DEBUG
        printf("INVALID CRC, received ");
        printf("%X", crc);
        printf(" but calculated ");
        printf("%X", crc_calc);
        #endif        
        return false;
      }    
    }
    else
    {
#ifndef LOCK_ONLY_ISO15693
        /* eject card if invalid */
        static bool need_reset = false;      
        static unsigned int eject_cooldown = 0;
        if (eject_cooldown > 0) eject_cooldown--;
        static unsigned long long eject_request_time = 0;
        if (((icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_BACK_ON) &&
          (icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_FRONT_ON) &&
          (icca_state.status_code & AC_IO_ICCA_SENSOR_NO_CARD))) 
        {
          #ifdef ICCX_DEBUG
          printf("bad card inside");
          #endif
            unsigned long long curr_time = to_ms_since_boot(get_absolute_time());
            if ((eject_request_time != 0) && (curr_time - eject_request_time >= EJECT_DELAY))
            {
              #ifdef ICCX_DEBUG
              printf("eject now!");
              #endif
              eject_cooldown = 20;
              if (!iccx_eject_card(AC_IO_ICCA_SLOT_STATE_CLOSE))
              {
              return false;
              }            
              eject_request_time = 0;
              need_reset = true;
            }
            else if ((eject_cooldown == 0) && (eject_request_time == 0))
            {
              #ifdef ICCX_DEBUG
              printf("request eject!");
              #endif
              eject_request_time = curr_time;
            }
        }
#endif

#ifdef LOCK_ONLY_ISO15693  
        /* allow new card to be inserted when currently inserting ISO15693 */
        if (((icca_state.status_code & AC_IO_ICCA_SENSOR_CARD)) && (!(icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_BACK_ON) ||
            !(icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_FRONT_ON)))
#else
        /* allow new card to be inserted when slot is clear */
        if (((!(icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_BACK_ON) &&
            !(icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_FRONT_ON))))
#endif
        {

#ifndef LOCK_ONLY_ISO15693
        if (need_reset)
          {
            if (!iccx_set_state(node_id, AC_IO_ICCA_SLOT_STATE_CLOSE, NULL)) 
            {
            return false;
            }
            need_reset = false;
          }
#endif
            if (!iccx_set_state(node_id, AC_IO_ICCA_SLOT_STATE_OPEN, NULL)) 
            {
            return false;
            }
        }
#ifdef LOCK_ONLY_ISO15693  
        else {
            if (!iccx_set_state(node_id, AC_IO_ICCA_SLOT_STATE_CLOSE, NULL)) 
            {
            return false;
            }
        }
#endif
        /* lock the card when fully inserted */
        if ((icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_BACK_ON) &&
          (icca_state.sensor_state & AC_IO_ICCA_SENSOR_MASK_FRONT_ON) &&
          (icca_state.status_code & AC_IO_ICCA_SENSOR_CARD))
        {
          if (!iccx_set_state(
                node_id, AC_IO_ICCA_SLOT_STATE_CLOSE, NULL)) 
          {
            return false;
          }
        }
    
      
    }

    if (state != NULL) {
        memcpy(state, msg.cmd.raw, sizeof(iccx_state_t));
        memcpy(&icca_state, msg.cmd.raw, sizeof(icca_state_t));
    }

    return true;
}

static bool iccx_read_card(uint8_t node_id, iccx_state_t *state, bool encrypted)
{

    struct ac_io_message msg;

    msg.addr = node_id + 1;
    if (encrypted)
    {
      msg.cmd.code = ac_io_u16(AC_IO_CMD_ICCx_FEL_ENGAGE);
      msg.cmd.nbytes = 4;
      static uint8_t fel_poll[4] = {0x00,0x03,0xFF,0xFF};
      memcpy(&msg.cmd.raw, fel_poll, 4);      
    }
    else
    {
      
      msg.cmd.code = ac_io_u16(AC_IO_CMD_ICCx_ENGAGE);
      msg.cmd.nbytes = 1;
      msg.cmd.count = sizeof(iccx_state_t);
    }
    
    if (!acio_send_and_recv(
            &msg, offsetof(struct ac_io_message, cmd.raw) + msg.cmd.count)) {
              #ifdef ICCX_DEBUG
        printf("Reading card of node ");
        printf("%d", node_id + 1);
        printf(" failed");
            #endif
        return false;
    }

    if (state != NULL) {
        memcpy(state, msg.cmd.raw, sizeof(iccx_state_t));
    }

    return true;
}


bool iccx_scan_card(uint8_t *type, uint8_t *uid, uint16_t *key_state, bool encrypted)
{

  iccx_state_t state;


//icca_read_card suivi de get_state

  #ifdef ICCX_DEBUG
   printf("STEP1. CARD READ");
  #endif
  if (!iccx_read_card(0, &state, encrypted)){
  #ifdef ICCX_DEBUG
   printf("cmd read card failed");
  #endif
  return false;
 }
  
  #ifdef ICCX_DEBUG
   printf("STEP2. GET STATE");
  #endif
 if (!iccx_get_state(0, &state, encrypted)){
  #ifdef ICCX_DEBUG
   printf("cmd get state failed");
  #endif
  return false;
 }

/* copy data into type and uid*/
 #ifdef ICCX_DEBUG
   printf("scan card success.");
   if (state.sensor_state == 2){
    if ((state.card_type&0x0F) == AC_IO_ICCx_CARD_TYPE_FELICA) printf("FeliCa ");
    else printf("ISO15693 ");
    printf("card found!");
    printf("UID =");
    for (int i=0; i<8; i++)
    {
      printf(" ");
      if (state.uid[i] < 0x10) printf("0");
      printf("%X\n", state.uid[i]);
    }
    printf("\n");
    printf("card type = ");
    printf("%d", state.card_type&0x0F);
   }
   else {
    printf("no card found (status = ");
    printf("%d", state.sensor_state);
    printf(")");
   }
  #endif
  
  *key_state = state.key_state;
  if (!encrypted)
  {
    if (state.card_type != 0x30){
      *type = 0;
      return true;
    }
  }
  if (state.sensor_state == AC_IO_ICCx_SENSOR_CARD) {
  memcpy(uid, state.uid, 8);
  *type = (state.card_type&0x0F)+1;
  }
  else *type = 0;
  
  return true;
  
}
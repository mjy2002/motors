//probablemente sea necesario crear dos funciones para enviar y recibir ya sea char o array

uint8_t envio(unsigned char data)
{
  uint8_t timeout = 1;
  uint8_t flag = 0;
  uint8_t i = 0;
  unsigned char temp[8];
  uint32_t numero = 0;
  unsigned char c = 0;
  memset(temp, 0, sizeof(temp));
  
  while(timeout <= 10)
  {
    if(flag == 0)
    {
      tx_string(data);
      //ACTIVA TIMER 
      flag = 1;
    }
    while(rxflag == 0)
    {
      if(//BANDERA DEL TIMER)
      {
        //DESACTIVA TIMER
        timeout++;
        flag = 0; 
        break;   
      }
    }
    if(rxflag == 1)
    {
      while (buffer[i] != 0x00)
      {
        temp[i] = buffer[i];
        i++;
      }
      if (i > 1)
      {
        // convertir temp en numero, convertir data en numero antes de compararlo
        if(numero == data)    //Probable necesidad de cast 
          flag = 1; 
      }
      else if (i == 1)
      {
        c = temp[1];
        if(c == data)
          flag = 1; 
      }
      else
      {
        flag = 0;
      }
    }
  }
  rxflag = 0;
  if(flag == 1)
    return 1;
  else
    return 0;
}


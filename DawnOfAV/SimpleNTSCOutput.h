#pragma once
#include "driver/i2s.h"

const int lineSamples = 910;

const int memSamples = 912;
const int syncSamples = 68;
const int burstSamples = 36;
const int burstStart = 76;
const int frameStart = 200; //must be even to simplify buffer word swap 136+(910-640-136)/2
const int imageSamples = 640;

const int syncLevel = 0;
const int blankLevel = 25;
const int burstAmp = 11;
const int maxLevel = 60;
//const double burstPerSample = (2 * M_PI) / (13333333 / 4433618.75);
const double burstPerSample = 1.5707963;
const float colorFactor = (M_PI * 2) / 16;
const float burstPhase = M_PI / 4 * 3;

class SimpleNTSCOutput
{
  public:
      
  unsigned short lineLongSync[memSamples];
  unsigned short lineShortSync[memSamples];
  unsigned short lineShortShortSync[memSamples];
  unsigned short lineHalfFrameSync[memSamples];
  unsigned short lineShortLongSync[memSamples];
  unsigned short lineLongShortSync[memSamples];
  unsigned short line[2][memSamples];
  unsigned short lineBlank[2][memSamples];
  short SIN[imageSamples];
  short COS[imageSamples];

  short YLUT[16];
  short UVLUT[16];

  static const i2s_port_t I2S_PORT = (i2s_port_t)I2S_NUM_0;
    
  SimpleNTSCOutput()
  {
    //template of sync signal
    for(int i = 0; i < memSamples; i++)
    {
      lineLongSync[i ^ 1] = syncLevel  << 8;
      lineShortSync[i ^ 1] = blankLevel << 8;
      //equalizing pulse
      lineShortShortSync[i ^ 1] = blankLevel << 8;
      //equalizing pulse(field B)
      lineShortLongSync[i ^ 1] = blankLevel << 8;

      lineHalfFrameSync[i ^ 1] = blankLevel << 8;
      lineLongShortSync[i ^ 1] = syncLevel << 8;
    }
    for(int i = 0; i < syncSamples/2; i++)
    {
      lineShortSync[i ^ 1] = syncLevel << 8;
      lineShortShortSync[i ^ 1] = syncLevel << 8;
      lineShortShortSync[(lineSamples / 2 + i) ^ 1] = syncLevel << 8;
      lineHalfFrameSync[(lineSamples / 2 + i) ^ 1] = syncLevel << 8;
      lineShortLongSync[i ^ 1] = syncLevel << 8;
    }
    
    for(int i = 0; i < syncSamples; i++)
    {
      lineLongSync[(lineSamples / 2 - syncSamples + i) ^ 1] = blankLevel  << 8;
      lineLongSync[(lineSamples - syncSamples + i) ^ 1] = blankLevel  << 8;
      lineHalfFrameSync[i ^ 1] = syncLevel << 8;
      lineLongShortSync[(lineSamples / 2 - syncSamples + i) ^ 1] = blankLevel  << 8;
      line[0][i ^ 1] = syncLevel  << 8;
      line[1][i ^ 1] = syncLevel  << 8;
      lineBlank[0][i ^ 1] = syncLevel  << 8;
      lineBlank[1][i ^ 1] = syncLevel  << 8;
    }
    
    for(int i = 0; i < lineSamples - syncSamples; i++)
    {
      if(i >= (lineSamples / 2 - syncSamples) && i < (lineSamples - syncSamples * 2)){
        lineShortLongSync[(i + syncSamples) ^ 1] = syncLevel  << 8;
      }
      if(i <= (lineSamples / 2 - syncSamples / 2)){
        lineLongShortSync[(lineSamples/2 + syncSamples / 2 + i) ^ 1] = blankLevel  << 8;
      }
      line[0][(i + syncSamples) ^ 1] = blankLevel  << 8;
      line[1][(i + syncSamples) ^ 1] = blankLevel  << 8;
      lineBlank[0][(i + syncSamples) ^ 1] = blankLevel  << 8;
      lineBlank[1][(i + syncSamples) ^ 1] = blankLevel  << 8;
    }
    for(int i = 0; i < burstSamples; i++)
    {
      int p = burstStart + i;
      unsigned short b0 = ((short)(blankLevel + sin(i * burstPerSample) * burstAmp)) << 8;
      unsigned short b1 = ((short)(blankLevel + sin(i * burstPerSample + M_PI) * burstAmp)) << 8;
      line[0][p ^ 1] = b0;
      line[1][p ^ 1] = b1;
      lineBlank[0][p ^ 1] = b0;
      lineBlank[1][p ^ 1] = b1;
    }
  
    for(int i = 0; i < imageSamples; i++)
    {
      int p = frameStart + i;
      int c = p - burstStart;
      SIN[i] = round(0.436798 * sin(c * burstPerSample) * 256);
      COS[i] = round(0.614777 * cos(c * burstPerSample) * 256);     
    }

    for(int i = 0; i < 16; i++)
    {
      YLUT[i] = (blankLevel << 8) + round(i / 15. * 256 * maxLevel);
      UVLUT[i] = round((i - 8) / 7. * maxLevel);
    }
  }

  void init()
  {

    i2s_config_t i2s_config = {
       .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
       .sample_rate = 1000000,  //not really used
       .bits_per_sample = (i2s_bits_per_sample_t)I2S_BITS_PER_SAMPLE_16BIT, 
       .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
       .communication_format = I2S_COMM_FORMAT_I2S_MSB,
       .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
       .dma_buf_count = 20,
       //.dma_buf_count = 10,
       .dma_buf_len = memSamples,   //a buffer per line
       .use_apll = 1,
       .tx_desc_auto_clear = 1,
       .fixed_mclk = 30000000,
       
    };
    
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);    //start i2s driver
    i2s_set_pin(I2S_PORT, NULL);                           //use internal DAC
    i2s_set_sample_rates(I2S_PORT, 1000000);               //dummy sample rate, since the function fails at high values
  
    //this is the hack that enables the highest sampling rate possible ~13MHz, have fun
//    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A_V, 1, I2S_CLKM_DIV_A_S);
//    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B_V, 1, I2S_CLKM_DIV_B_S);
//    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM_V, 2, I2S_CLKM_DIV_NUM_S); 
//    SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_TX_BCK_DIV_NUM_V, 2, I2S_TX_BCK_DIV_NUM_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A_V, 21, I2S_CLKM_DIV_A_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B_V, 1, I2S_CLKM_DIV_B_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM_V, 1, I2S_CLKM_DIV_NUM_S); 
    SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_TX_BCK_DIV_NUM_V, 1, I2S_TX_BCK_DIV_NUM_S);

    //untie DACs
    //SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(0), I2S_TX_CHAN_MOD_V, 3, I2S_TX_CHAN_MOD_S);
    //SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), I2S_TX_FIFO_MOD_V, 1, I2S_TX_FIFO_MOD_S);
  }

  void sendLine(unsigned short *l)
  {
    esp_err_t error = ESP_OK;
    size_t bytes_written = 0;
    size_t bytes_to_write = lineSamples * sizeof(unsigned short);
    size_t cursor = 0;
    while (error == ESP_OK && bytes_to_write > 0) {
      error = i2s_write(I2S_PORT, (char*)l + cursor, bytes_to_write, &bytes_written, portMAX_DELAY);
      bytes_to_write -= bytes_written;
      cursor += bytes_written;
    }
  }

  void sendFrame(char ***frame)
  {
    static int field = 0;
    static int nextfield[] = { 3, 0, 1, 2};
    
    //equalizing pulses
      sendLine(lineShortShortSync);
      sendLine(lineShortShortSync);
      if(field == 0 || field == 2){
        sendLine(lineShortShortSync);
      }else{
        sendLine(lineShortLongSync);
      }
    //v-sync pulses
      sendLine(lineLongSync);
      sendLine(lineLongSync);
      if(field == 0 || field == 2){
        sendLine(lineLongSync);
      }else{
        sendLine(lineLongShortSync);
      }      
    //equalizing pulses
      sendLine(lineShortShortSync);
      sendLine(lineShortShortSync);
      if(field == 0 || field == 2){
        sendLine(lineShortShortSync);
      }else{
        sendLine(lineShortSync);
      }
    //blank lines
      if(field <= 1){
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
      }else{
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
        sendLine(lineBlank[0]);
        sendLine(lineBlank[1]);
      }
    //image
    for(int i = 0; i < 240; i += 2)
    {
      char *pixels0 = (*frame)[i];
      char *pixels1 = (*frame)[i + 1];
      int j = frameStart;
      if(field <= 1){
        for(int x = 0; x < imageSamples; x += 2){
          unsigned short p0 = *(pixels0++);
          unsigned short p1 = *(pixels1++);
          short y0 = YLUT[p0 & 15];
          short y1 = YLUT[p1 & 15];
          short p04 = UVLUT[p0 >> 4];
          short p14 = UVLUT[p1 >> 4];
          short u0 = (SIN[x] * p04);
          short u1 = (SIN[x + 1] * p04);
          short v0 = (COS[x] * p14);
          short v1 = (COS[x + 1] * p14);
          //word order is swapped for I2S packing (j + 1) comes first then j
          line[1][j] = y0 + u1 + v1;
          line[0][j] = y1 - u1 - v1;
          line[1][j+1] = y0 + u0 + v0;
          line[0][j+1] = y1 - u0 - v0;
          j += 2;
        }
        sendLine(line[1]);
        sendLine(line[0]); 
      }else{
        for(int x = 0; x < imageSamples; x += 2){
          unsigned short p0 = *(pixels0++);
          unsigned short p1 = *(pixels1++);
          short y0 = YLUT[p0 & 15];
          short y1 = YLUT[p1 & 15];
          short p04 = UVLUT[p0 >> 4];
          short p14 = UVLUT[p1 >> 4];
          short u0 = (SIN[x] * p04);
          short u1 = (SIN[x + 1] * p04);
          short v0 = (COS[x] * p14);
          short v1 = (COS[x + 1] * p14);
          //word order is swapped for I2S packing (j + 1) comes first then j
          line[0][j] = y0 - u1 - v1;
          line[1][j] = y1 + u1 + v1;
          line[0][j+1] = y0 - u0 - v0;
          line[1][j+1] = y1 + u0 + v0;
          j += 2;
        }
        sendLine(line[0]);
        sendLine(line[1]);
      }
    }
    if(field <= 1){
      sendLine(lineBlank[1]);
      sendLine(lineBlank[0]);
    }else{
      sendLine(lineBlank[0]);
      sendLine(lineBlank[1]);
    }
    if(field == 0 || field == 2){
      sendLine(lineHalfFrameSync);
    }
    
    field = nextfield[field];
  }
};

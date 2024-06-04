/*
 * CS - to digital pin 10  (SS pin)
 * SDI - to digital pin 11 (MOSI pin)
 * CLK - to digital pin 13 (SCK pin)
 */

// Include SPI library
#include <SPI.h>
// #include <DueTimer.h>

// 執行次數
unsigned int run_times = 10;
unsigned int run_times_temp = 0;

/*設定點座標，最末點後給255*/
// X
// Y
// const int set_point[2][85]{
//  { -160 ,-144 ,-128 ,-112 ,-96 ,-80 ,-64 ,-48 ,-32 ,-16 ,0 ,16 ,32 ,48 ,64 ,80 ,96 ,112 ,128 ,144 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,144 ,128 ,112 ,96 ,80 ,64 ,48 ,32 ,16 ,0 ,-16 ,-32 ,-48 ,-64 ,-80 ,-96 ,-112 ,-128 ,-144 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,255},
// { 160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,160 ,144 ,128 ,112 ,96 ,80 ,64 ,48 ,32 ,16 ,0 ,-16 ,-32 ,-48 ,-64 ,-80 ,-96 ,-112 ,-128 ,-144 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-160 ,-144 ,-128 ,-112 ,-96 ,-80 ,-64 ,-48 ,-32 ,-16 ,0 ,16 ,32 ,48 ,64 ,80 ,96 ,112 ,128 ,144 ,160 ,255},
//};

const int set_point[2][112]{
    {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 150, 120, 90, 60, 30, 0, -30, -60, -90, -120, -150,
     -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 150, 120, 90, 60, 30, 0, -30, -60, -90, -120, -150,
     -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 150, 120, 90, 60, 30, 0, -30, -60, -90, -120, -150,
     -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 150, 120, 90, 60, 30, 0, -30, -60, -90, -120, -150,
     -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 150, 120, 90, 60, 30, 0, -30, -60, -90, -120, -150, 255},
    {-20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
     30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60,
     90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60,
     30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255},
};

/*設定每點移動時間間隔 ms，最大值65535*/
unsigned int t_flag = 2;

// set pin 10 as the CS pin
const int CS_Pin = 10;
// set pin 6 as the FCLK_X pin
// set pin 8 as the FCLK_Y pin
const int FCLK_X = 6, FCLK_Y = 8;

// set pin 4 as Control pin
const int ctl_pin = 4;
// set pin 5 as Output Enable pin
const int OutEn_pin = 5;

// set V bias
int V_bias = 90;
double V_bias_dig = V_bias / 200.0 * 65535.0;
int V_bias_dig0 = V_bias_dig;
int V_range = 60;

unsigned int t_index = 1;
unsigned int point_index = 0;

void setup()
{
  // set the CS Pin as an output:
  Serial.begin(9600);
  // Pin 13 indicate the output button signal: LED "L" light up/on; LED "L" dark/off
  pinMode(13, OUTPUT);
  delay(1);
  pinMode(24, OUTPUT);
  delay(1);
  pinMode(26, OUTPUT);
  delay(1);
  pinMode(28, OUTPUT);
  delay(1);
  digitalWrite(24, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, LOW);

  pinMode(CS_Pin, OUTPUT);
  delay(1);
  digitalWrite(CS_Pin, HIGH);
  delay(1);
  pinMode(OutEn_pin, OUTPUT);
  delay(1);
  digitalWrite(OutEn_pin, LOW);
  delay(100);
  pinMode(ctl_pin, INPUT_PULLUP);
  delay(100);

  pinMode(FCLK_X, OUTPUT);
  pinMode(FCLK_Y, OUTPUT);

  Timer3.attachInterrupt(FCLK);
  Timer3.start(16.6667); // Calls every 30kHz

  Timer4.attachInterrupt(sig_out);
  Timer4.stop();
  // Timer4.start(1000); // Calls every 1kHz

  // initialize SPI:
  SPI.begin();

  SPI.setDataMode(SPI_MODE2);

  DA_Write(0x28, 0x00, 0x01);
  delay(100);
  DA_Write(0x38, 0x00, 0x01);
  delay(100);
  DA_Write(0x20, 0x00, 0x0F);
  delay(100);
  DA_Write(0x30, 0x00, 0x0F);
  delay(100);
}

double i_temp = 0.0;
unsigned char H8T = 0, M8T = 0, L8T = 0;

unsigned int ctl_push_times = 0, CS_status = 1;
void loop()
{
  // Serial.println(L8T);
  if (digitalRead(ctl_pin) == HIGH)
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);

  if ((digitalRead(ctl_pin) == HIGH) && (CS_status == 0))
  {
    digitalWrite(13, HIGH);

    if (ctl_push_times > 10)
    {
      run_times_temp = 0;
      point_index = 0;
      Timer4.start(1000);
      digitalWrite(OutEn_pin, HIGH);
      CS_status = 1;
      ctl_push_times = 0;
    }
    else
      ctl_push_times++;
  }
  else if ((digitalRead(ctl_pin) == LOW) && (CS_status == 1))
  {
    if (ctl_push_times > 10)
    {
      Timer4.stop();
      BackZero(set_point[0][point_index], set_point[1][point_index]);
      delay(50);
      digitalWrite(OutEn_pin, LOW);
      point_index = 0;
      CS_status = 0;
      ctl_push_times = 0;
    }
    else
      ctl_push_times++;
  }
  else if (CS_status == 2)
  {
    BackZero(set_point[0][point_index], set_point[1][point_index]);
    digitalWrite(OutEn_pin, LOW);
    point_index = 0;
    ctl_push_times = 0;
    CS_status = 1;
  }
  delay(25);
  // Serial.println(run_times_temp);
}

void DA_Write(unsigned char H8, unsigned char M8, unsigned char L8)
{
  // take the SS pin low to select the chip:
  digitalWrite(CS_Pin, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(H8);
  SPI.transfer(M8);
  SPI.transfer(L8);
  // take the SS pin high to de-select the chip:
  digitalWrite(CS_Pin, HIGH);
}

bool SigOn = false;
void FCLK()
{
  SigOn = !SigOn;

  digitalWrite(FCLK_X, SigOn);
  digitalWrite(FCLK_Y, SigOn);
}

void sig_out()
{

  if (run_times > run_times_temp)
  {

    if (t_index < t_flag)
      t_index++;
    else
    {
      if (set_point[0][point_index] >= 0)
      {
        i_temp = 1.0 * set_point[0][point_index];
        i_temp = i_temp / 200.0 * 65535.0 / 2.0;
        // H8T=0x10;
        H8T = 0x10;
        // M8T=(i>>8)&0x00FF;
        M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);

        H8T = 0x11;
        M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);
      }
      else
      {
        i_temp = -1.0 * set_point[0][point_index];
        i_temp = i_temp / 200.0 * 65535.0 / 2.0;
        // H8T=0x10;
        H8T = 0x10;
        // M8T=(i>>8)&0x00FF;
        M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);

        H8T = 0x11;
        M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);
      }

      if (set_point[1][point_index] >= 0)
      {
        i_temp = 1.0 * set_point[1][point_index];
        i_temp = i_temp / 200.0 * 65535.0 / 2.0;
        H8T = 0x12;
        M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);

        H8T = 0x13;
        M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);
      }
      else
      {
        i_temp = -1.0 * set_point[1][point_index];
        i_temp = i_temp / 200.0 * 65535.0 / 2.0;
        H8T = 0x12;
        M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);

        H8T = 0x13;
        M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
        L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
        DA_Write(H8T, M8T, L8T);
      }

      t_index = 1;

      if ((set_point[0][point_index + 1] >= 180) || (set_point[1][point_index + 1] >= 180) || (set_point[0][point_index + 1] < -180) || (set_point[1][point_index + 1] < -180))
      {
        point_index = 0;
        digitalWrite(28, HIGH);
        delayMicroseconds(2);
        digitalWrite(28, LOW);
        run_times_temp++;
      }
      else
        point_index++;

      if ((point_index == 0) || ((point_index > 0) && (set_point[1][point_index] != set_point[1][point_index - 1])))
      {
        digitalWrite(26, HIGH);
        // delayMicroseconds(100);
        digitalWrite(26, LOW);
      }

      digitalWrite(24, HIGH); // 2us
      // delayMicroseconds(10);
      digitalWrite(24, LOW);
    }
  }
  else
  {
    CS_status = 2;
    Timer4.stop();
  }
}

void BackZero(int now_pointX, int now_pointY)
{
  while ((now_pointX != 0) || (now_pointY != 0))
  {
    if (now_pointX > 5)
      now_pointX = now_pointX - 5;
    else if (now_pointX < -5)
      now_pointX = now_pointX + 5;
    else if ((now_pointX <= 5) && (now_pointX >= -5))
      now_pointX = 0;

    if (now_pointX >= 0)
    {
      i_temp = 1.0 * now_pointX;
      i_temp = i_temp / 200.0 * 65535.0 / 2.0;

      H8T = 0x10;
      M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);

      H8T = 0x11;
      M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);
    }
    else
    {
      i_temp = -1.0 * now_pointX;
      i_temp = i_temp / 200.0 * 65535.0 / 2.0;

      H8T = 0x10;
      M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);

      H8T = 0x11;
      M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);
    }

    if (now_pointY > 5)
      now_pointY = now_pointY - 5;
    else if (now_pointY < -5)
      now_pointY = now_pointY + 5;
    else if ((now_pointY <= 5) && (now_pointY >= -5))
      now_pointY = 0;

    if (now_pointY >= 0)
    {
      i_temp = 1.0 * now_pointY;
      i_temp = i_temp / 200.0 * 65535.0 / 2.0;

      H8T = 0x12;
      M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);

      H8T = 0x13;
      M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);
    }
    else
    {
      i_temp = -1.0 * now_pointY;
      i_temp = i_temp / 200.0 * 65535.0 / 2.0;
      H8T = 0x12;
      M8T = ((V_bias_dig0 - (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 - (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);

      H8T = 0x13;
      M8T = ((V_bias_dig0 + (unsigned int)i_temp) / 256) & 0x00FF;
      L8T = ((V_bias_dig0 + (unsigned int)i_temp) % 256) & 0x00FF;
      DA_Write(H8T, M8T, L8T);
    }
    delay(5);
  }
  delay(100);
}

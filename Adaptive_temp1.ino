// Setup
float kf = 0.01;              // Constante de filtrado
const int t_sample = 200;     // Tiempo de filtro
const int periodo_t = 20;
const int hz = 3;             // Horizonte de prediccion (5 o menos)
const float n = 9.0;              // Periodos de control para alcanzar el SP (9)
float MaxOut_Temp;
const float UP_Temp = 255;    // salida maxima del controlador
const float GainT_Temp = 80;  // Ganancia del controlador
const int PmA = 2;            // Delay Parameters a
const int PmB = 2;            // Delay Parameters b
const float NL = 0.01;          // Noise Level for Adaptive Mechanism.
const float GainA = 1.0;         // Gain for Adaptive Mechanism A
const float GainB = 1.0;         // Gain for Adaptive Mechanism B

// Variables
float temp_sf;                        // Temperatura sin filtrar
float temp_fil;                       // Temperatura filtrada
const unsigned char sensor_pin = 0;   // Pin sensor LM35 
const int motor_pin = 11;             // Pin salida PWM motor
unsigned int periodo;             // Periodo Control = Periodo * t_sample
float outTemp;                        // Salida controlador
char input[10];                    // Input set point


// Variables del bloque conductor
static double r1[hz] = {0};                // param for desired out conductor block
static double r2[hz] = {0};
double s1[hz] = {0};
static double s2[hz] = {0};
static double w = 0;

// Variables control adaptativo
float yt[PmA+2] = {0};            // Array process out Temp  y(k), y(k-1), y(k-2), y(k-3)
float ut[PmB+5] = {0};           // Array process input Temp  u(k), u(k-1), u(k-2), u(k-3), u(k-4)  
float tt[7] = {1.0, -0.5, 0.1, 0.1, 0.1, 0.1, 0.1};  // Array parameters adaptive mechanism a1k, a2k, b1k, b2k, b3k    
//double tp[7] = {0, 0, 0, 0, 0, 0, 0};
float spt[2] = {0};            // Set point process Temp sp(k), sp(k-1), sp(k-2)  
static unsigned char ini = 0;

void setup() {
  
  Serial.begin(115200);
  analogReference(INTERNAL);        // Referencia en 1.1V   
  // Primera lectura

  
  for (int i = 4; i > 0; i--){        // Inicializa las lecturas
    temp_sf = analogRead(sensor_pin);  
    temp_sf = (1.1 * temp_sf) / 10,23;    
    temp_fil = temp_sf;
    }
  conductor_block();                  // Calculo del bloque conductor
  periodo = 0;    
  MaxOut_Temp = UP_Temp/GainT_Temp;                 // Calculo de la salida maxima del controlador      
  /*
  Serial.println(r1[hz-1]);
  Serial.println(r2[hz-1]);
  Serial.println(s2[hz-1]);
  Serial.println(w); 
  Serial.println("\r");
*/
 
}

void loop() {

  periodo++;                                    // Cuenta periodos de control       
  temp_sf = (1.1 * analogRead(sensor_pin)) / 10.23;  // Leemos la temperatura y se convierte a grados
  temp_fil = lpf(temp_sf, temp_fil, kf);

  // Leer set point del puerto serie
  while(Serial.available()>0){
    int i;
    input[i] = Serial.read();
    i++;
  }
  spt[0]=atoi(input);
  if (periodo >= periodo_t) {
    
    // Control adaptativo
    periodo = 0;                  // Reset de periodos de control           
    //spt[0] = 29;                // Set point    
    yt[0] = temp_fil;                                // Process Out Temp y(k). 
    adaptive(spt, tt, yt, ut, MaxOut_Temp); 
    outTemp = -ut[1] * GainT_Temp;  
    
  }

  Serial.println(temp_fil);
  //Serial.println(ut[0]);
  Serial.println(outTemp);
  Serial.println(spt[0]);

  Serial.println("\r");

  analogWrite(motor_pin, outTemp);

  delay(t_sample);
}


// Filtro paso bajo 
float lpf(float pv_sf, float pv_fil_ant, float kf){
  float pv_fil;
  pv_fil = (kf * pv_sf + (1 - kf) * pv_fil_ant);    //pv_fil = kf * pv_sf + (1 - kf) * pv_fil_ant;
  return pv_fil;
}

// Bloque conductor Calculo segun amortiguamiento critico y ganancia unitaria
void conductor_block(void){
  float a1,a2,b1,b2;
  
  a1 = 2 * exp(-6.0/n);
  a2 = -exp(-12.0/n);
  b1 = 1 - (1 + 6.0/n) * exp(-6.0/n);
  b2 = 1 - a1 - a2 - b1;
  
  r1[0] = a1; // Vector e1 para cada horizonte a1
  r2[0] = a2; // vector e2 para cada horizonte a2
  s1[0] = b1; // vector g1 para cada horizonte b1
  s2[0] = b2; // vector g2 para cada horizonte b2
  w = 0;
  
  for (int j = 1; j < hz; j++){
    r1[j] = r1[j-1] * r1[0] + r2[j-1];
    r2[j] = r1[j-1] * r2[0];
    s1[j] = r1[j-1] * s1[0] + s2[j-1];
    s2[j] = r1[j-1] * s2[0];
    }
  for (int j = 0; j < hz; j++){
    w += s1[j];
    }
  // Parameters for conductor block
  // r1[hz-1] * y(k)
  // r2[hz-1] * y(k-1)
  // s2[hz-1] * sp(k-1)
  // w    * sp(k)
  }

  // Adaptive Control no incremental order 2
void adaptive(float *sp, float *t, float *y, float *u, float max_out){
  float y_k;       // Estimated out
  float ek;        // Estimation error
  float q;       // Aux for adaptive process
  float y_dk;      //  Desired Out
  unsigned char adap;     // Enable/disable  adaptation
    
  if (ini <= (PmA+10)) ini++;   // Counter for initialize  input/output vector    
  
  if (ini >= (PmA+10)){     // Control for initialize  input/output vector  
    // Adaptive model order 2 with 3 parameters b
    y_k = t[0] * y[PmA] + t[1] * y[PmA+1] + t[2] * u[PmB] + t[3] * u[PmB+1] + t[4] * u[PmB+2] + t[5] * u[PmB+3] + t[6] * u[PmB+4];    // Estimated out Delay Process = 1
    ek = (y[0] - y_k);                                        // Estimation error 
    if (fabs(ek) > NL) adap = 1;                                  // Noise Level
      else adap = 0;

    //Adaptive mechanism
    q = adap * ek / (1.0 + (GainA * (pow(y[PmA],2) + pow(y[PmA+1],2)) + GainB * (pow(u[PmB],2) + pow(u[PmB+1],2) + pow(u[PmB+2],2) + pow(u[PmB+3],2) + pow(u[PmB+4],2))));  
    t[0] += (GainA * q * y[PmA]);
    t[1] += (GainA * q * y[PmA+1]);
    t[2] += (GainB * q * u[PmB]);
    t[3] += (GainB * q * u[PmB+1]);
    t[4] += (GainB * q * u[PmB+2]);
    t[5] += (GainB * q * u[PmB+3]);
    t[6] += (GainB * q * u[PmB+4]); 
        
    y_dk = r1[hz-1] * yt[0] + r2[hz-1] * yt[1] + s2[hz-1] * sp[1] + w * sp[0];   // Desired out calculated. Conductor block. Model ref: a1, a2, b1, b2                 
    
  
    // Calculated parameters extended strategy
    double e1[hz] = { t[0] }; // Vector e1 for horizon
    double e2[hz] = { t[1] }; // vector e2 for horizon      
    double g1[hz] = { t[2] }; // vector g1 for horizon    
    double g2[hz] = { t[3] }; // vector g2 for horizon    
    double g3[hz] = { t[4] }; // vector g3 for horizon
    double g4[hz] = { t[5] }; // vector g3 for horizon
    double g5[hz] = { t[6] }; // vector g3 for horizon             
    double h = 0;       // h parameter
    
    for (int j = 1; j < hz; j++){
        e1[j] = e1[j-1] * e1[0] + e2[j-1];
        e2[j] = e1[j-1] * e2[0];        
      g1[j] = e1[j-1] * g1[0] + g2[j-1];
      g2[j] = e1[j-1] * g2[0] + g3[j-1];
      g3[j] = e1[j-1] * g3[0] + g4[j-1];
      g4[j] = e1[j-1] * g4[0] + g5[j-1];
      g5[j] = e1[j-1] * g5[0];                
    }
    for (int j = 0; j < hz; j++){     
      h += g1[j];     
    }   
    u[0] = (y_dk - e1[hz-1] * y[0] - e2[hz-1] * y[1] - g2[hz-1] * u[1] - g3[hz-1] * u[2] - g4[hz-1] * u[3] - g5[hz-1] * u[4]) / h;    // Calculated Regulator Out
                
    if (u[0] < - max_out) u[0] = - max_out;           // Upper limit out
      else if (u[0] > 0) u[0] = 0;
  } // End counter for init output/input vector 
  //save_process for K+1  
  for (int i = (PmA+1); i > 0; i--){  
    y[i] = y[i-1];
  } 
  
  for (int i = (PmB+4); i > 0; i--){
    u[i] = u[i-1];
  }       
  sp[1] = sp[0];
   
}




#include "third-party/include/serial/serial.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <math.h>
#include <string>
#include "thread"
using namespace std;
int MAX_SAMPLES = 25, a = 0;
char *serial_dato;
std::ofstream file;
std::string acc_vect, dato;
string a_data[3];
float a_data_s[3] = {0,0,0};
float a_data_prom[3] = {0,0,0};
int cont_samples = 0;
string menu;
bool flag = false;
int main()
{
    serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(3000));
   
     file.open("/home/kiqmc/accel_calib_data/accel_data.txt");

    if (my_serial.isOpen())
    {
        std::cout << "Port opened succesfully" << std::endl;
    }
    else
    {
        std::cout << "Port failed to open" << std::endl;
        return 0;
    }
    cout << "Mantenga el sensor quieto ..." << endl;


    while (1)
    {   

        
        cout << "Presione una tecla para confirmar" << endl;
        cin >> menu;
        
        cont_samples = 0;
        a_data_s[0] = 0;
        a_data_s[1] = 0;
        a_data_s[2] = 0;
        
        
        for (int m = 0; m < MAX_SAMPLES ; m++){
                
            a_data[0].clear();
            a_data[1].clear();
            a_data[2].clear();

              
            read_serial:

            my_serial.flushInput(); // Limpieza del buffer del serial
            dato.clear();
            dato = my_serial.readline();

            if(dato[0]!='s') // Se garantiza que la cadena empieza con el dato correcto
                goto read_serial;
            
            
            a = 0;

            if (dato.size() >= 26){
                
                for (int i = 1; i < dato.size()-1; i++){
                    
                    if (dato[i] != ','){
                        a_data[a] += dato[i];
                    }
                    else  {  
                        a++; 
                    }
                }
                cont_samples++;
                a_data_s[0]+= stof(a_data[0]);
                a_data_s[1]+= stof(a_data[1]);
                a_data_s[2]+= stof(a_data[2]);
            }
        }

        a_data_prom[0] = a_data_s[0]/(cont_samples);
        a_data_prom[1] = a_data_s[1]/(cont_samples);
        a_data_prom[2] = a_data_s[2]/(cont_samples);
        
        float magnitude = sqrt(pow(a_data_prom[0],2) + pow(a_data_prom[1],2)+pow(a_data_prom[2],2));
        if (magnitude < 1.1 && magnitude > 0.9){
            cout << a_data_prom[0] << ',' << a_data_prom[1]<<',' << a_data_prom[2]<< "   M: "<< magnitude<< " - Muestras realizadas: "<< cont_samples<<  endl;
            file << a_data_prom[0] << ' ' << a_data_prom[1]<<' ' << a_data_prom[2]<<  endl;
        }
        else{
            cout << "Datos no guardados, intente nuevamente, magnitud del vector "<< magnitude << endl;
        }
    }

    my_serial.close();
    file.close();
    return 0;
}
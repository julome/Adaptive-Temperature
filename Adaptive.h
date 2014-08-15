/*********************************************************************
			Library Adaptive predictive control
			Copyright (C) 2014  Juan Lopez Medina

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	You can contact me in julome21@gmail.com

    Robot balancer Adaptive  Copyright (C) 2013  Juan Lopez Medina
	
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.
 ****************************************************************************/ 

#ifndef ADAPTIVE_H_
#define ADAPTIVE_H_



#define PmA				2			// Delay Parameters a
#define PmB				2			// Delay Parameters b
#define n				10.0		// Conductor block periods control for rise to set point ts = n * CP
#define hz				4			// Prediction Horizon (Horizon max = n + 2)
#define UP_PWM			190			// Upper limit out

void conductor_block(void);
void adaptive(double *sp, double *t, double *y, double *u, double *yp, int max_out);

#endif /* ADAPTIVE_H_ */

// Define variables Adaptive Predictive control Configuration
const extern double Ref_Meters;		// Reference for meters in adaptive predictive control
extern double NL;					// Noise Level for Adaptive Mechanism
extern double GainA; 				// Gain for Adaptive Mechanism A
extern double GainB; 				// Gain for Adaptive Mechanism B
extern double MaxOut_PWM; 



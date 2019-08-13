#include "motion.h"
#include <iostream>
#include <unistd.h>

using namespace std;

int main(){
	motion_init();
	double t;
	motion_set_x_speed(166.667);
	//motion_set_y_speed(0);
	//double freq = motion_get_f();
	for (int i = 0; i < 51; ++i)
	{
		t = i*0.02;

		cout << "t: " << t << endl;
		
		motion_tick(t);
		cout << "l1： " << l1[0] << endl;
		cout << "l2： " << l2[0] << endl;
		cout << "l3： " << l3[0] << endl;
	}
	//cout << "freq: " << freq <<endl;
	cout << "dx " << motion_get_dx() <<endl;
	cout << "dy " << motion_get_dy() <<endl;
	cout << "turn " << motion_get_turn() <<endl;
	return 0;
}

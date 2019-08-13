#include <jni.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "HoloBinding.h"
#include "Holobot.h"
using namespace std;
using namespace Metabot;

Holobot * the_bot = NULL;

/*
 * Class:     HoloBinding
 * Method:    init
 * Signature: (Ljava/lang/String;I)Z
 */
JNIEXPORT jboolean JNICALL Java_HoloBinding_init
(JNIEnv * env, jobject obj, jstring port, jint baud) {
  const char * port_str = env->GetStringUTFChars(port, NULL);
  printf("- initialisation of holobot system on port %s with baudrate : %d\n", port_str, baud);
  the_bot = new Holobot(std::string(port_str), baud);
  if (the_bot == NULL) {
    printf("==> fatal error\n");
  }
  return true;
}

/*
 * Class:     HoloBinding
 * Method:    close
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_HoloBinding_close
(JNIEnv *, jobject) {
  if (the_bot != NULL) {
    printf("- closing holobot system\n");
    delete the_bot;
  }
  the_bot = NULL;
  return true;
}

/*
 * Class:     HoloBinding
 * Method:    monitor
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_HoloBinding_monitor
(JNIEnv *, jobject, jboolean activate) {
  if (the_bot == NULL) return;
  the_bot->debug_state(activate);
}

/*
 * Class:     HoloBinding
 * Method:    beep
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_HoloBinding_beep
(JNIEnv * env, jobject obj, jint freq, jint duration) {
  if (the_bot == NULL) return;
  the_bot->beep((short) freq, (short) duration);
}

/*
 * Class:     HoloBinding
 * Method:    get_time
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_HoloBinding_get_1time
(JNIEnv *, jobject) {
  if (the_bot == NULL) return 0.0;
  return the_bot->get_time();
}

/*
 * Class:     HoloBinding
 * Method:    get_dist
 * Signature: (I)F
 */
JNIEXPORT jfloat JNICALL Java_HoloBinding_get_1dist
(JNIEnv *, jobject, jint idx) {
  if (the_bot == NULL) return 0.0;
  return the_bot->get_dist(idx);
}

/*
 * Class:     HoloBinding
 * Method:    get_opt
 * Signature: (I)F
 */
JNIEXPORT jfloat JNICALL Java_HoloBinding_get_1opt
(JNIEnv *, jobject, jint idx) {
  if (the_bot == NULL) return 0.0;
  return the_bot->get_opt(idx);
}

/*
 * Class:     HoloBinding
 * Method:    get_yaw
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_HoloBinding_get_1yaw
(JNIEnv *, jobject) {
  if (the_bot == NULL) return 0.0f;
  return the_bot->get_yaw();
}

/*
 * Class:     HoloBinding
 * Method:    reset_yaw
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_HoloBinding_reset_1yaw
(JNIEnv *, jobject) {
  if (the_bot == NULL) return;
  return the_bot->reset_yaw();
}

/*
 * Class:     HoloBinding
 * Method:    get_wheel_speeds
 * Signature: (I)F
 */
JNIEXPORT jfloat JNICALL Java_HoloBinding_get_1wheel_1speeds
(JNIEnv *, jobject, jint idx) {
  if (the_bot == NULL) return 0.0;
  return the_bot->get_wheel_speeds(idx);
}

/*
 * Class:     HoloBinding
 * Method:    control
 * Signature: (FFF)V
 */
JNIEXPORT void JNICALL Java_HoloBinding_control
(JNIEnv *, jobject, jfloat dx, jfloat dy, jfloat turn) {
  if (the_bot == NULL) return;
  the_bot->control(dx,dy,turn);
}

/*
 * Class:     HoloBinding
 * Method:    move_toward
 * Signature: (FF)V
 */
JNIEXPORT void JNICALL Java_HoloBinding_move_1toward
(JNIEnv *, jobject, jfloat speed, jfloat direction) {
  if (the_bot == NULL) return;
  the_bot->move_toward(speed, direction);
}

/*
 * Class:     HoloBinding
 * Method:    turn
 * Signature: (F)V
 */
JNIEXPORT void JNICALL Java_HoloBinding_turn
(JNIEnv *, jobject, jfloat rot_speed) {
  if (the_bot == NULL) return;
  the_bot->turn(rot_speed);  
}

/*
 * Class:     HoloBinding
 * Method:    stop_all
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_HoloBinding_stop_1all
(JNIEnv *, jobject) {
  if (the_bot == NULL) return;
  the_bot->stop_all();
}

/*
 * Class:     HoloBinding
 * Method:    print_state
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_HoloBinding_print_1state
(JNIEnv *, jobject){
  if (the_bot == NULL) return;
  the_bot->print_state();
} 

/*
 * Class:     HoloBinding
 * Method:    set_board_led
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_HoloBinding_set_1board_1led
(JNIEnv *, jobject, jint state) {
  if (the_bot == NULL) return;
  the_bot->set_board_led((uint8_t) state);
}


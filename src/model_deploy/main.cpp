#include "accelerometer_handler.h"
#include "tfconfig.h"
#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "mbed.h"
#include "mbed_rpc.h"

#include "uLCD_4DGL.h"

#include "stm32l475e_iot01_accelero.h"

#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#include <cmath>

uLCD_4DGL uLCD(D1, D0, D2); // serial tx, serial rx, reset pin;

RpcDigitalOut myled3(LED3, "myled3");
RpcDigitalOut myled2(LED2, "myled2");
RpcDigitalOut myled1(LED1, "myled1");
DigitalIn sw2(USER_BUTTON);
EventQueue queue(32 * EVENTS_EVENT_SIZE);

Thread t;

BufferedSerial pc(USBTX, USBRX);
void GUI_Thread(Arguments *in, Reply *out);
void DET_Thread(Arguments *in, Reply *out);
RPCFunction GUI(&GUI_Thread, "GUI_Thread");
RPCFunction DET(&DET_Thread, "DET_Thread");
double x, y;

int selection_flag = 0;
int i = 1; //judge select or not
float gravity_x = 0.0f;
float gravity_y = 0.0f;
float gravity_z = 0.0f;
float arccos = 0.0f;

int event_num = 0;
char buffer_send[3][100];

WiFiInterface *wifi;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

const char* topic = "Mbed";
//NetworkInterface* net = wifi;
//MQTTNetwork mqttNetwork(net);
//MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
//const char* host = "192.168.1.103";
//SocketAddress sockAddr;

//Thread mqtt_thread(osPriorityHigh);
//EventQueue mqtt_queue;

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    message_num++;
    MQTT::Message message;
    char buff[100];
    //sprintf(buff, "QoS0 Hello, Python! #%d", message_num);
    for(int k = 0; k<3; k++){
    strcpy(buff, buffer_send[k]);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);

    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
    }
}

void close_mqtt() {
    closed = true;
}

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

void normal_mode();
void mode_select1();
void mode_select();
void mode_up();
void mode_down();
void GUI_temp();
int mqtt();
  int mqtt(){
    

    
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }

    
    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.1.103";

    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    //mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    //sw2.rise(mqtt_queue.event(&publish_message, &client));
    //btn3.rise(&close_mqtt);

    publish_message(&client);

    /*int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }

    while (1) {
            if (closed) break;
            client.yield(500);
            ThisThread::sleep_for(500ms);
    }*/
    //if(event_num == 2){
    printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
    //}
    return 0;
  }

// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

void mode_down()
{
    selection_flag = (selection_flag+1)%4;
    if(selection_flag == 0)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
    else if(selection_flag == 1)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
    else if(selection_flag == 2)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
    else if(selection_flag == 3)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
}


void mode_select()
{
    //led2 = !led2;
    i = 0;
    if(selection_flag == 0)
    {        
        uLCD.text_width(2); 
        uLCD.text_height(2);
        uLCD.color(RED);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");
        printf("You select 15 degree.\n");
    }
    else if(selection_flag == 1)
    {
        uLCD.text_width(2); 
        uLCD.text_height(2);
        uLCD.color(RED);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");
        printf("You select 30 degree.\n");
    }
    else if(selection_flag == 2)
    {
        uLCD.text_width(2); 
        uLCD.text_height(2);
        uLCD.color(RED);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");
        printf("You select 45 degree.\n");
    }
    else if(selection_flag == 3)
    {
        uLCD.text_width(2); 
        uLCD.text_height(2);
        uLCD.color(RED);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
        printf("You select 60 degree.\n");
    }
}

void mode_up()
{
    selection_flag = (selection_flag+3)%4;
    if(selection_flag == 0)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
    else if(selection_flag == 1)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
    else if(selection_flag == 2)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
    else if(selection_flag == 3)
    {
        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,0);
        uLCD.printf("\n  15");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,2);
        uLCD.printf("\n  30");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(WHITE);
        uLCD.locate(2,4);
        uLCD.printf("\n  45");

        uLCD.text_width(2); //4X size text
        uLCD.text_height(2);
        uLCD.color(GREEN);
        uLCD.locate(2,6);
        uLCD.printf("\n  60");
    }
}


int GUI_mode() {

  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return -1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return -1;
  }

  error_reporter->Report("Set up successful...\n");

  //t.start(callback(&queue, &EventQueue::dispatch_forever));
  normal_mode();
  
  while (i == 1) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    //t.start(callback(&queue, &EventQueue::dispatch_forever));
    //normal_mode();
    if (gesture_index < label_num) {
      //error_reporter->Report(config.output_message[gesture_index]);
      if(gesture_index == 0)
      {
        //queue.call(mode_down);
        mode_down();
      }
      else if(gesture_index == 1)
      {
        //queue.call(mode_up);
        mode_up();
      }
      //sw2.rise(queue.event(mode_select));
    }
    if(sw2==0) {
      mode_select();
      return 1;
    }
    //sw2.rise(queue.event(mode_select));

  }
}


int main(){
      //The mbed RPC classes are now wrapped to create an RPC enabled version - see RpcClasses.h so don't add to base class

    // receive commands, and send back the responses
    char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    //normal_mode();
    //t.start(callback(&queue, &EventQueue::dispatch_forever));
    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }
}

void DET_mode_init(){
    //char buff[100];
    int16_t pDataXYZ[3] = {0};
    BSP_ACCELERO_Init();
    for(int j = 0; j < 10 ; j++)
    {
        BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        //printf("%d %d %d\n\r",pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
        gravity_x = gravity_x + float(pDataXYZ[0]);
        gravity_y = gravity_y + float(pDataXYZ[1]);
        gravity_z = gravity_z + float(pDataXYZ[2]);
        ThisThread::sleep_for(100ms);
    }
    gravity_x = gravity_x/10;
    gravity_y = gravity_y/10;
    gravity_z = gravity_z/10;
    printf("%4f %4f %4f\n\r", gravity_x, gravity_y, gravity_z);
}

int DET_mode(){
  int16_t pDataXYZ[3] = {0};
  BSP_ACCELERO_Init(); 
  //int event_num = 0;
  float target_angle;
  if(selection_flag == 0) target_angle = 15.0;
  else if(selection_flag == 1) target_angle = 30.0;
  else if(selection_flag == 2) target_angle = 45.0;
  else if(selection_flag == 3) target_angle = 60.0;
  while(1){
      BSP_ACCELERO_AccGetXYZ(pDataXYZ);
      float x_temp = float(abs(pDataXYZ[0]));
      float z_temp = float(abs(pDataXYZ[2]));
      float temp = (x_temp*gravity_x+z_temp*gravity_z)/((sqrt(gravity_x*gravity_x+gravity_z*gravity_z))*sqrt(x_temp*x_temp+z_temp*z_temp));
      arccos = acos(temp)*180/M_PI;
      //printf("%.1f\n\r",arccos);

      uLCD.cls();
      uLCD.background_color(BLACK);
      uLCD.text_width(2); //4X size text
      uLCD.text_height(2);
      uLCD.color(WHITE);
      uLCD.locate(2,2);
      uLCD.printf("%.1f",arccos);

      if(arccos >= target_angle){
        sprintf(buffer_send[event_num],"Tha angle is %.1f, which is larger than %.1f\r\n",arccos,target_angle);
        printf("%s",buffer_send[event_num]);
        //mqtt();
        if(event_num == 2) {
          mqtt();
          return 1;
        }
        event_num++;
      }
      ThisThread::sleep_for(100ms);
  }
}
void DET_Thread(Arguments *in, Reply *out){
  bool success = true;

    // In this scenario, when using RPC delimit the two arguments with a space.
    x = in->getArg<double>();
    y = in->getArg<double>();

    // Have code here to call another RPC function to wake up specific led or close it.
    char buffer[200], outbuf[256];
    char strings[20];
    int led = x;
    int on = y;
    sprintf(strings, "/myled%d/write %d", led, on);
    strcpy(buffer, strings);
    RPC::call(buffer, outbuf);

    //t.start(callback(&queue, &EventQueue::dispatch_forever));
    if (success) {
        out->putData(buffer);
        DET_mode_init();
        sprintf(strings, "/myled%d/write 0", led);
        strcpy(buffer, strings);
        RPC::call(buffer, outbuf);
        sprintf(strings, "/myled1/write 1");
        strcpy(buffer, strings);
        RPC::call(buffer, outbuf);
        DET_mode();
        sprintf(strings, "/myled1/write 0");
        strcpy(buffer, strings);
        RPC::call(buffer, outbuf);
        //mqtt();
    } else {
        out->putData("Failed to execute LED control.");
    }
}

void GUI_Thread(Arguments *in, Reply *out){
    bool success = true;

    // In this scenario, when using RPC delimit the two arguments with a space.
    x = in->getArg<double>();
    y = in->getArg<double>();

    // Have code here to call another RPC function to wake up specific led or close it.
    char buffer[200], outbuf[256];
    char strings[20];
    int led = x;
    int on = y;
    sprintf(strings, "/myled%d/write %d", led, on);
    strcpy(buffer, strings);
    RPC::call(buffer, outbuf);

    t.start(callback(&queue, &EventQueue::dispatch_forever));
    if (success) {
        out->putData(buffer);
        GUI_mode();
    } else {
        out->putData("Failed to execute LED control.");
    }
}

/*void GUI_temp(){
  queue.call(GUI_mode);
}*/

void normal_mode()
{
    uLCD.background_color(WHITE);
    uLCD.color(BLUE);
    //uLCD.text_width(1); //4X size text
    //uLCD.text_height(1);
    //uLCD.printf("\nSelect the slew rate\n");

    uLCD.text_width(2); //4X size text
    uLCD.text_height(2);
    uLCD.color(GREEN);
    uLCD.locate(2,0);
    uLCD.printf("\n  15");

    uLCD.text_width(2); //4X size text
    uLCD.text_height(2);
    uLCD.color(WHITE);
    uLCD.locate(2,2);
    uLCD.printf("\n  30");

    uLCD.text_width(2); //4X size text
    uLCD.text_height(2);
    uLCD.color(WHITE);
    uLCD.locate(2,4);
    uLCD.printf("\n  45");

    uLCD.text_width(2); //4X size text
    uLCD.text_height(2);
    uLCD.color(WHITE);
    uLCD.locate(2,6);
    uLCD.printf("\n  60");
}


void mode_select1(){
  queue.call(mode_select);
}
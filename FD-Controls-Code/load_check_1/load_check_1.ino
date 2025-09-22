

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

}


float sensor_data[4] = {0,0,0,0}; //This will be updated every frame
bool is_broken[4] = {false, false, false, false}; //Updated in load_check1 function


// Method 1: Find max, and check if any sensor val is way less than that
void load_check1() {
  int max_ind = 0;
  float thresh = 0.25;
  for (int i = 0; i < 4; i++) {
    if (sensor_data[i] > sensor_data[max_ind]) {max_ind = i;}
  }
  for (int i = 0; i < 4; i++) {
    is_broken[i] = (sensor_data[i] <= thresh * sensor_data[max_ind]);
  }
}


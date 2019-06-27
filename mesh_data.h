#ifndef MESH_DATA_H
#define MESH_DATA_H

#define ALARM_ON                   0x03
#define ALARM_OFF                  0x00

#define MAX_UNICAST_ADDRESS        63

typedef struct {
	uint8 alarm_signal;
	uint16 unicast_address;
	uint8 heart_beat;
	uint8 battery_percent;
} mesh_data_t;
uint16 set_mesh_data(mesh_data_t mesh_data) {
	uint16 data = 0x0000;
	data = data | (mesh_data.alarm_signal & 0x03);
	data = data | ((mesh_data.unicast_address & 0x3f) << 2);
	data = data | ((mesh_data.heart_beat & 0x01) << 8);
	data = data | ((mesh_data.battery_percent & 0x7f) << 9);
	return data;
}
mesh_data_t get_mesh_data(uint16 data) {
	mesh_data_t mesh_data;
	mesh_data.alarm_signal = data & 0x03;
	mesh_data.unicast_address = (data >> 2) & 0x3f;
	mesh_data.heart_beat = (data >> 8) & 0x01;
	mesh_data.battery_percent = (data >> 9) & 0x7f;
	return mesh_data;
};
typedef struct {
	uint8 status;
	uint16 address;
	uint8 timeOut;
} lpn_status;
uint16 get_lpn_status_index(uint16 address, lpn_status* array, uint16 num_lpn){
	uint8 i = 0;
	for(;i< num_lpn; i++ ){
		if(array[i].address == address){
			return i;
		}
	}
	printf("Can't find the address");
}
uint8 is_friend_or_gateway_address(uint16 address, lpn_status* array, uint16 num_lpn){
	uint8 i = 0;
	for(;i< num_lpn; i++){
		if(array[i].address == address){
			return 1;
		}
		else return 0;
	}
}

void refresh_timeOut_and_status(uint16 address, lpn_status* array, uint16 num_lpn){
	uint8 i = 0;
	for(;i < num_lpn; i++){
		if(array[i].address == address)
			array[i].status = 1;
			array[i].timeOut = 0;
	}
}
void refine_lpn_status_arr(uint16 terminate_this_address, lpn_status* array, uint16 num_lpn) {
	uint8 result_index = get_lpn_status_index(terminate_this_address,
			array, num_lpn);
		array[result_index].address = array[num_lpn].address;
		array[result_index].status = array[num_lpn].status;
		array[result_index].timeOut = array[num_lpn].timeOut;
	num_lpn--;
}

#endif

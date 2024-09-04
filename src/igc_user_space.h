
#ifndef _IGC_USER_SPACE_API_H_
#define _IGC_USER_SPACE_API_H_

#include <stdint.h>

bool     igc_user_space_supported();
void     igc_user_space_init(uint32_t rx_timeout_us);
void     igc_user_space_deinit();
void     igc_user_space_get_mac(uint8_t* data);
bool     igc_user_space_get_link_status();
void     igc_user_space_send_frame(uint8_t* data, int len);
uint32_t igc_user_space_receive_frame(uint8_t* receive_pkt);

#endif /* _IGC_USER_SPACE_API_H_ */
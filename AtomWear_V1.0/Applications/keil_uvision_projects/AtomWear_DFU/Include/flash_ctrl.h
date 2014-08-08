#ifndef _FLASH_CTRL_H
#define _FLASH_CTRL_H

void system_app_pages_erase(uint32_t page_address_base, uint8_t pages_count);

uint8_t system_page_refresh(uint8_t cur_page, uint8_t *page_data, uint32_t data_length);

void flash_specword_write(uint32_t *address, uint8_t offset, uint32_t data);

#endif

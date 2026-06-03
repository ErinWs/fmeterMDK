
// modbus exception code
// 0x01 Illegal Function
// 0x02 Illegal Data Address
// 0x03 Illegal Data Value
// 0x04 Slave Device Failure

// Supported function codes 0x03 0x06 0x10

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MD_MODBUS_RINGBUF_SIZE          256U  /* must be power of two */
#define MD_MODBUS_FRAME_TIMEOUT_MS      200

#if (MD_MODBUS_RINGBUF_SIZE & (MD_MODBUS_RINGBUF_SIZE - 1)) != 0
#error "MD_MODBUS_RINGBUF_SIZE must be a power of two"
#endif


typedef void (*modbus_tx_cb_t)(const uint8_t *buf, uint16_t len);
typedef void (*modbus_write_cb_t)(uint16_t addr, uint16_t count, const uint16_t *data);

static modbus_tx_cb_t modbus_tx_cb = NULL;
static modbus_write_cb_t modbus_write_cb = NULL;

static struct modbus_misc_t
{
    uint8_t  ringbuf[MD_MODBUS_RINGBUF_SIZE];
    uint16_t read_idx;
    uint16_t write_idx;
    uint16_t send_buf[8];

    uint16_t frame_byte_time_out;
    uint16_t frame_time_out;
    union sw_t
    {
        uint16_t all;
        struct _bit_t
        {
            uint8_t frame_active : 1;
            uint8_t running : 1;
        } _bit;
    } sw;

    modbus_tx_cb_t tx_cb;
    modbus_write_cb_t write_cb;
} modbus_misc = {0};

static void modbus_ringbuf_init(void)
{
    modbus_misc.read_idx = 0;
    modbus_misc.write_idx = 0;
    memset((void *)modbus_misc.ringbuf, 0, sizeof(modbus_misc.ringbuf));
}

static bool modbus_ringbuf_is_empty(void)
{
    return modbus_misc.read_idx == modbus_misc.write_idx;
}

static uint16_t modbus_ringbuf_available(void)
{
    return (uint16_t)((modbus_misc.write_idx + MD_MODBUS_RINGBUF_SIZE - modbus_misc.read_idx) & (MD_MODBUS_RINGBUF_SIZE - 1));
}

static bool modbus_ringbuf_push_byte(uint8_t b)
{
    uint16_t next = (uint16_t)((modbus_misc.write_idx + 1) & (MD_MODBUS_RINGBUF_SIZE - 1));
    if (next == modbus_misc.read_idx)
    {
        return false;
       // modbus_misc.read_idx = (modbus_misc.read_idx + 1) & (MD_MODBUS_RINGBUF_SIZE - 1);
    }
    modbus_misc.ringbuf[modbus_misc.write_idx] = b;
    modbus_misc.write_idx = next;
    return true;
}

static uint8_t modbus_ringbuf_peek(uint16_t offset)
{
    return modbus_misc.ringbuf[(modbus_misc.read_idx + offset) & (MD_MODBUS_RINGBUF_SIZE - 1)];
}

static void modbus_ringbuf_consume(uint16_t n)
{
    modbus_misc.read_idx = (uint16_t)((modbus_misc.read_idx + n) & (MD_MODBUS_RINGBUF_SIZE - 1));
}

static uint8_t modbus_ringbuf_get_byte(void)
{
    uint8_t b = 0;
    if (!modbus_ringbuf_is_empty())
    {
        b = modbus_misc.ringbuf[modbus_misc.read_idx];
        modbus_misc.read_idx = (uint16_t)((modbus_misc.read_idx + 1) & (MD_MODBUS_RINGBUF_SIZE - 1));
    }
    return b;
}




static uint16_t crc16_modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else crc >>= 1;
        }
    }
    return crc;
}

static void modbus_send_exception(uint8_t addr, uint8_t func, uint8_t ex_code)
{
    if (addr == 0x00) return; /* broadcast: no response */
    if (modbus_misc.tx_cb == NULL) return;
    uint8_t *resp=modbus_misc.send_buf;
    resp[0] = addr;
    resp[1] = func | 0x80;
    resp[2] = ex_code;
    uint16_t crc = crc16_modbus(resp, 3);
    resp[3] = (uint8_t)(crc & 0xFF);
    resp[4] = (uint8_t)(crc >> 8);
    modbus_misc.tx_cb(resp, 5);
}


static void modbus_init(modbus_tx_cb_t tx,modbus_write_cb_t wr)
{
    modbus_misc.tx_cb = tx;
    modbus_misc.write_cb = wr;
    modbus_ringbuf_init();
}

static void modbus_feed_byte_isr(uint8_t b)
{
    (void)modbus_ringbuf_push_byte(b);
}

static uint16_t modbus_read_reg(uint16_t addr)
{
   // if (addr < holding_count) return holding_ptr[addr];
   // return 0;
}

static void modbus_write_reg(uint16_t addr, uint16_t val)
{
   // if (addr < holding_count) holding_ptr[addr] = val;
}

static uint16_t peek_u16(uint16_t off)
{
    uint8_t hi = modbus_ringbuf_peek(off);
    uint8_t lo = modbus_ringbuf_peek(off + 1);
    return ((uint16_t)hi << 8) | lo;
}

static void modbus_50ms_tick_task(void)
{
    if(mosbus_misc.frame_time_out>0)
    {
        mosbus_misc.frame_time_out--;
    }

    if(modbus_misc.frame_byte_time_out>0)
    {
        modbus_misc.frame_byte_time_out--;
        if(modbus_misc.frame_byte_time_out==0)
        {

        }
    }
}

static uint16_t modbus_try_once(void)
{
    uint16_t avail = modbus_ringbuf_available();
    if (avail < 8) return 0;
    uint16_t offset = 0;
    while (offset <= (uint16_t)(avail - 8)) {
        uint8_t a = modbus_ringbuf_peek(offset);
        if (a == MB_SLAVE_ADDR || a == 0x00) break;
        offset++;
    }
    if (offset) {
        modbus_ringbuf_consume(offset);
        avail = modbus_ringbuf_available();
        if (avail < 8) return 0;
    }

    if (!modbus_misc.sw._bit.frame_active)
    {
        modbus_misc.sw._bit.frame_active = 1;
        modbus_misc.frame_time_out=10;
    }
    else
    {
        if(modbus_misc.frame_time_out==0)
        {
            modbus_ringbuf_consume(1);
            modbus_misc.sw._bit.frame_active = 0;
            return 1;
        }
    }

    uint8_t addr = modbus_ringbuf_peek(0);
    uint8_t func = modbus_ringbuf_peek(1);

    /* Fixed-length requests 03/06 */
    if (func == 0x03 || func == 0x06) {
        if (avail < 8) return 0;
        uint8_t frame[8];
        for (int i = 0; i < 8; i++) frame[i] = modbus_ringbuf_peek(i);

        uint16_t crc_calc = crc16_modbus(frame, 6);
        uint16_t crc_in = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
        if (crc_calc != crc_in) {
            /* CRC 错误：静默丢弃一个字节以重同步 */
            modbus_ringbuf_consume(1);
            modbus_misc.sw._bit.frame_active = 0;
            return 1;
        }

        uint16_t reg = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t v_or_qty = ((uint16_t)frame[4] << 8) | frame[5];
        int need_resp = (addr != 0x00);

        if (func == 0x03) {
            uint16_t qty = v_or_qty;
            if (qty == 0 || qty > 125 || (uint32_t)reg + qty > holding_count) {
                modbus_send_exception(addr, 0x03, 0x02); /* Illegal Data Address */
                modbus_ringbuf_consume(8);
                modbus_misc.sw._bit.frame_active = 0;
                return 8;
            }
            if (need_resp && tx_cb) {
                uint16_t payload = qty * 2;
                uint16_t len = 3 + payload + 2;
                uint8_t *resp = (uint8_t *)malloc(len);
                if (resp) {
                    uint16_t idx = 0;
                    resp[idx++] = MB_SLAVE_ADDR;
                    resp[idx++] = 0x03;
                    resp[idx++] = (uint8_t)payload;
                    for (uint16_t i = 0; i < qty; i++) {
                        uint16_t val = holding_ptr[reg + i];
                        resp[idx++] = (uint8_t)(val >> 8);
                        resp[idx++] = (uint8_t)(val & 0xFF);
                    }
                    uint16_t crc = crc16_modbus(resp, idx);
                    resp[idx++] = (uint8_t)(crc & 0xFF);
                    resp[idx++] = (uint8_t)(crc >> 8);
                    tx_cb(resp, idx);
                    free(resp);
                }
            }
            modbus_ringbuf_consume(8);
            modbus_misc.sw._bit.frame_active = 0;
            return 8;
        } else { /* 0x06 */
            if (reg >= holding_count) {
                modbus_send_exception(addr, 0x06, 0x02); /* Illegal Data Address */
                modbus_ringbuf_consume(8);
                modbus_misc.sw._bit.frame_active = 0;
                return 8;
            }
            holding_ptr[reg] = v_or_qty;
            if (write_cb) write_cb(reg, 1, &v_or_qty);
            if (need_resp && tx_cb) tx_cb(frame, 8); /* echo */
            modbus_ringbuf_consume(8);
            modbus_misc.sw._bit.frame_active = 0;
            return 8;
        }
    }
    else if (func == 0x10) {
        /* 需要至少 7 字节 header 才能读到 bytecount */
        if (avail < 7) return 0;
        uint16_t reg_addr = peek_u16(2);
        uint16_t qty = peek_u16(4);
        uint8_t bytecnt = modbus_ringbuf_peek(6);

        /* 如果 qty 明显非法（0 或超上限），直接丢弃 1 字节以重同步（不发送异常） */
        if (qty == 0 || qty > 125) {
            modbus_ringbuf_consume(1);
            modbus_misc.sw._bit.frame_active = 0;
            return 1;
        }

        /* 如果 header 中的 bytecnt 与 qty*2 不匹配，同样不立即发异常，丢 1 字节重同步 */
        if (bytecnt != (uint8_t)(qty * 2)) {
            modbus_ringbuf_consume(1);
            modbus_misc.sw._bit.frame_active = 0;
            return 1;
        }

        uint16_t full_len = (uint16_t)(7 + bytecnt + 2); /* header + data + CRC */
        if (avail < full_len) return 0; /* 等待完整帧（或被超时打断） */

        /* 现在收齐，读取并校验 CRC */
        uint8_t *frame = (uint8_t *)malloc(full_len);
        if (!frame) { modbus_ringbuf_consume(full_len); modbus_misc.sw._bit.frame_active = 0; return full_len; }
        for (uint16_t i = 0; i < full_len; i++) frame[i] = modbus_ringbuf_peek(i);
        uint16_t crc_calc = crc16_modbus(frame, (uint16_t)(full_len - 2));
        uint16_t crc_in = (uint16_t)frame[full_len - 2] | ((uint16_t)frame[full_len - 1] << 8);
        if (crc_calc != crc_in) {
            free(frame);
            /* CRC 错误：静默丢弃 1 字节以重同步（不发异常） */
            modbus_ringbuf_consume(1);
            modbus_misc.sw._bit.frame_active = 0;
            return 1;
        }

        /* CRC 合格后再根据地址/范围决定写入或发异常 */
        int need_resp = (frame[0] != 0x00);
        uint16_t base = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];

        if ((uint32_t)base + count > holding_count) {
            /* 只有在 CRC 合格并确认是完整帧时才返回异常 */
            modbus_send_exception(frame[0], 0x10, 0x02); /* Illegal Data Address */
            free(frame);
            modbus_ringbuf_consume(full_len);
            modbus_misc.sw._bit.frame_active = 0;
            return full_len;
        }

        /* 写寄存器并回调/响应 */
        for (uint16_t i = 0; i < count; i++) {
            uint16_t vv = (uint16_t)frame[7 + i*2] << 8 | frame[7 + i*2 + 1];
            holding_ptr[base + i] = vv;
        }
        if (write_cb) write_cb(base, count, (const uint16_t *)&frame[7]);

        if (need_resp && tx_cb) {
            uint8_t resp[8];
            resp[0] = frame[0];
            resp[1] = 0x10;
            resp[2] = frame[2];
            resp[3] = frame[3];
            resp[4] = frame[4];
            resp[5] = frame[5];
            uint16_t crc = crc16_modbus(resp, 6);
            resp[6] = (uint8_t)(crc & 0xFF);
            resp[7] = (uint8_t)(crc >> 8);
            tx_cb(resp, 8);
        }

        free(frame);
        modbus_ringbuf_consume(full_len);
        modbus_misc.sw._bit.frame_active = 0;
        return full_len;
    }
    else {
        /* Illegal Function */
        modbus_send_exception(addr, func, 0x01); /* Illegal Function */
        modbus_ringbuf_consume(1);
        modbus_misc.sw._bit.frame_active = 0;
        return 1;
    }
}

void modbus_poll_task(void)
{
    while (1) 
    {
        uint16_t consumed = modbus_try_once();
        if (consumed == 0)
        {
            break;
        } 
    }
}
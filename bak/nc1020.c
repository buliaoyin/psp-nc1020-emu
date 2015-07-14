#include "nc1020.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

// cpu cycles per second (cpu freq).
#define CYCLES_SECOND 5120000
#define TIMER0_FREQ 2
#define TIMER1_FREQ 0x100
// cpu cycles per timer0 period (1/2 s).
#define CYCLES_TIMER0 (CYCLES_SECOND / TIMER0_FREQ)
// cpu cycles per timer1 period (1/256 s).
#define CYCLES_TIMER1 (CYCLES_SECOND / TIMER1_FREQ)
// speed up
#define CYCLES_TIMER1_SPEED_UP (CYCLES_SECOND / TIMER1_FREQ / 20)
// cpu cycles per ms (1/1000 s).
#define CYCLES_MS (CYCLES_SECOND / 1000)

// RAM/ROM page size
#define BANK_SIZE 0x8000
#define ROM_VOLUME_COUNT 0x3
#define ROM_VOLUME_BANK_COUNT 0x100
#define NOR_BANK_COUNT 0x20
#define ROM_SIZE (BANK_SIZE * ROM_VOLUME_COUNT * ROM_VOLUME_BANK_COUNT)
#define NOR_SIZE (BANK_SIZE * NOR_BANK_COUNT)
#define ROM_CACHE_COUNT 0x20
#define NOR_CACHE_COUNT 0x20

static const uint16_t IO_LIMIT = 0x40;
#define IO_API
typedef uint8_t (IO_API *io_read_func_t)(uint8_t);
typedef void (IO_API *io_write_func_t)(uint8_t, uint8_t);

static const uint16_t NMI_VEC = 0xFFFA;
static const uint16_t RESET_VEC = 0xFFFC;
static const uint16_t IRQ_VEC = 0xFFFE;

#define VERSION 0x06

typedef struct {
	uint16_t reg_pc;
	uint8_t reg_a;
	uint8_t reg_ps;
	uint8_t reg_x;
	uint8_t reg_y;
	uint8_t reg_sp;
} cpu_t;

typedef struct {
	size_t version;
	cpu_t cpu;
	uint8_t ram[0x8000];

	uint8_t bak_40[0x40];

	uint8_t clock_data[80];
	uint8_t clock_flags;

	uint8_t jg_wav_data[0x20];
	uint8_t jg_wav_flags;
	uint8_t jg_wav_idx;
	bool jg_wav_playing;

	uint8_t fp_step;
	uint8_t fp_type;
	uint8_t fp_bank_idx;
	uint8_t fp_bak1;
	uint8_t fp_bak2;
	uint8_t fp_buff[0x100];

	bool slept;
	bool should_wake_up;
	bool pending_wake_up;
	uint8_t wake_up_flags;

	bool timer0_toggle;
	size_t cycles;
	size_t timer0_cycles;
	size_t timer1_cycles;
	bool should_irq;

	size_t lcd_addr;
	uint8_t keypad_matrix[8];
} nc1020_t;

typedef struct {
    unsigned rom_hit;
    unsigned rom_miss;
    unsigned nor_hit;
    unsigned nor_miss;
    uint8_t rom_priority[ROM_CACHE_COUNT];
    uint8_t nor_priority[NOR_CACHE_COUNT];
    uint16_t rom_reverse[ROM_CACHE_COUNT];
    uint8_t nor_reverse[ROM_CACHE_COUNT];
} cache_ctrl_t;

static char nc1020_file_rom[256];
static char nc1020_file_nor[256];
static char nc1020_file_nor_tmp[256];
static char nc1020_file_sts[256];
static FILE* f_rom = NULL;
static FILE* f_nor = NULL;
static nc1020_t nc1020;

static uint8_t rom_cache[BANK_SIZE * ROM_CACHE_COUNT];
static uint8_t nor_cache[BANK_SIZE * NOR_CACHE_COUNT];
static uint8_t tmp_buf[BANK_SIZE];
static cache_ctrl_t cache_ctrl;

static uint8_t rom_bank[ROM_VOLUME_COUNT * ROM_VOLUME_BANK_COUNT];
static uint8_t* rom_volume[4];

static uint8_t nor_bank[NOR_BANK_COUNT];
/* bank: value&0xFF00>>8, page: value&0xF0>>4, ram_page: value&0x0F */
static uint16_t bbs_pages[0x10];

static uint8_t* memmap[8];

static uint8_t* ram_buff;
static uint8_t* stack;
static uint8_t* ram_io;
static uint8_t* ram_40;
static uint8_t* ram_page0;
static uint8_t* ram_page1;
static uint8_t* ram_page2;
static uint8_t* ram_page3;

static io_read_func_t io_read[0x40];
static io_write_func_t io_write[0x40];

static inline uint8_t GetRomCacheIdx(uint8_t vol, uint8_t bank_idx) {
    return rom_volume[vol][bank_idx];
}

static inline uint8_t GetNorCacheIdx(uint8_t bank_idx) {
    return nor_bank[bank_idx]&0x7F;
}

static inline uint8_t* GetRomCacheAddr(uint8_t vol, uint8_t bank_idx) {
    return &rom_cache[rom_volume[vol][bank_idx]*BANK_SIZE];
}

static inline uint8_t* GetNorCacheAddr(uint8_t bank_idx) {
    return &nor_cache[(nor_bank[bank_idx]&0x7F)*BANK_SIZE];
}

static void SaveNor(){
    int i = 0;

    for (i=0; i<NOR_BANK_COUNT; i++) {
        if (nor_bank[i] & 0x80)
            break;
    }
    if (i >= NOR_BANK_COUNT)
        return;

    long offset = 0;
    FILE* f_tmp = fopen(nc1020_file_nor_tmp, "wb+");
    printf("nor cache dirty, need flush!\n");
    for (i=0; i<NOR_BANK_COUNT; i++) {
        if (nor_bank[i] & 0x80) {
            fwrite(GetNorCacheAddr(i) + 0x4000, 0x4000, 1, f_tmp);
            fwrite(GetNorCacheAddr(i), 0x4000, 1, f_tmp);
            nor_bank[i] &= 0x7F;
        }
        else {
            fseek(f_nor, offset, SEEK_SET);
            fread(&tmp_buf, BANK_SIZE, 1, f_nor);
            fwrite(&tmp_buf, BANK_SIZE,1, f_tmp);
        }
        offset += BANK_SIZE;
    }

    fflush(f_tmp);
    fclose(f_tmp);

    fclose(f_nor);
    remove(nc1020_file_nor);
    rename(nc1020_file_nor_tmp, nc1020_file_nor);
    f_nor = fopen(nc1020_file_nor, "rb");
}

static void LoadRomBank(uint8_t vol, uint8_t bank_idx, bool force)
{
    int i;
    uint8_t max = 0, idx = 0;

    /* priority++ */
    for (i=0; i<ROM_CACHE_COUNT; i++) {
        if ((force || cache_ctrl.rom_priority[i]) && cache_ctrl.rom_priority[i] < 0xFE)
            cache_ctrl.rom_priority[i]++;
    }

    /* cache hit */
    if (rom_volume[vol][bank_idx] != 0xFF) {
        cache_ctrl.rom_hit++;
        cache_ctrl.rom_priority[GetRomCacheIdx(vol,bank_idx)] = force?0:1;
        //printf("rom want: %d:%d, hit, hit:%u, miss:%u\n", vol, bank_idx, cache_ctrl.rom_hit, cache_ctrl.rom_miss);
        return;
    }

    cache_ctrl.rom_miss++;
    for (i=0; i<ROM_CACHE_COUNT; i++) {
        if (cache_ctrl.rom_priority[i] >= max) {
            max = cache_ctrl.rom_priority[i];
            idx = i;
        }
    }

    /* no valid cache bank */
    if (max == 0) {
        printf("Fatal! No valid cache bank!\n");
        assert(0);
    }

    if (cache_ctrl.rom_priority[idx] < 0xFF) {
        rom_volume[cache_ctrl.rom_reverse[idx]&0xFF00>>8][cache_ctrl.rom_reverse[idx]&0xFF] = 0xFF;
    }
    fseek(f_rom, (rom_volume[vol]-rom_bank) * BANK_SIZE, SEEK_SET);
    fread(tmp_buf, BANK_SIZE, 1, f_rom);
    memcpy(&rom_cache[idx * BANK_SIZE], tmp_buf + 0x4000, 0x4000);
    memcpy(&rom_cache[idx * BANK_SIZE + 0x4000], tmp_buf, 0x4000);
    cache_ctrl.rom_priority[idx] = force?0:1;
    cache_ctrl.rom_reverse[idx] = bank_idx;
    rom_volume[vol][bank_idx] = idx;
    printf("rom want: %d:%d, miss, hit:%u, miss:%u\n", vol, bank_idx, cache_ctrl.rom_hit, cache_ctrl.rom_miss);
}

static void LoadNorBank(uint8_t bank_idx, bool force)
{
    int i;
    uint8_t max = 0, idx = 0;

    /* priority++ */
    for (i=0; i<NOR_CACHE_COUNT; i++) {
        if ((force || cache_ctrl.nor_priority[i]) && cache_ctrl.nor_priority[i] < 0xFE)
            cache_ctrl.nor_priority[i]++;
    }

    /* cache hit */
    if (nor_bank[bank_idx] != 0xFF) {
        cache_ctrl.nor_hit++;
        cache_ctrl.nor_priority[GetNorCacheIdx(bank_idx)] = force?0:1;
        //printf("nor want: %d, hit, hit:%u, miss:%u\n", bank_idx, cache_ctrl.nor_hit, cache_ctrl.nor_miss);
        return;
    }

    cache_ctrl.nor_miss++;
    for (i=0; i<NOR_CACHE_COUNT; i++) {
        if (cache_ctrl.nor_priority[i] >= max) {
            max = cache_ctrl.nor_priority[i];
            idx = i;
        }
    }

    if (cache_ctrl.nor_priority[idx] < 0xFF) {
        /* dirty? */
        SaveNor();
        nor_bank[cache_ctrl.nor_reverse[idx]] = 0xFF;
    }
    fseek(f_nor, bank_idx * BANK_SIZE, SEEK_SET);
    fread(tmp_buf, BANK_SIZE, 1, f_nor);
    memcpy(&nor_cache[idx * BANK_SIZE], tmp_buf + 0x4000, 0x4000);
    memcpy(&nor_cache[idx * BANK_SIZE + 0x4000], tmp_buf, 0x4000);
    cache_ctrl.nor_priority[idx] = force?0:1;
    cache_ctrl.nor_reverse[idx] = bank_idx;
    nor_bank[bank_idx] = idx;
    printf("nor want: %d, miss, hit:%u, miss:%u\n", bank_idx, cache_ctrl.nor_hit, cache_ctrl.nor_miss);
}

static uint8_t* GetBank(uint8_t bank_idx)
{
	uint8_t vol = 0x03 & ram_io[0x0D];
    if (bank_idx < 0x20) {
        LoadNorBank(bank_idx, false);
    	return GetNorCacheAddr(bank_idx);
    } else if (bank_idx >= 0x80) {
        LoadRomBank(vol, bank_idx, false);
        return GetRomCacheAddr(vol, bank_idx);
    }
    return NULL;
}

static void SwitchBank(){
	uint8_t bank_idx = ram_io[0x00];
	uint8_t* bank = GetBank(bank_idx);
    memmap[2] = bank;
    memmap[3] = bank + 0x2000;
    memmap[4] = bank + 0x4000;
    memmap[5] = bank + 0x6000;
}

static uint8_t* GetAddrFromBbs(uint8_t idx)
{
	uint8_t vol = 0x03 & ram_io[0x0D];
	uint16_t bbspage = bbs_pages[idx];
	if (bbspage & 0x1)
		return ram_buff + (bbspage & 0xF0 >> 4) * 0x2000;
	else {
		uint8_t bank_idx = bbspage&0xFF00 >> 8;
		LoadRomBank(vol, bank_idx, true);
		return GetRomCacheAddr(vol, bank_idx) + (bbspage & 0xF0 >> 4) * 0x2000;
	}
}

static void SwitchVolume(){
	int i;
	uint8_t volume_idx = 0x03 & ram_io[0x0D];
	uint8_t roa_bbs = ram_io[0x0A];
    for (i=0; i<4; i++) {
        bbs_pages[i * 4 + 0] = (uint16_t)i<<8;
        bbs_pages[i * 4 + 1] = ((uint16_t)i<<8) + (0x1<<4);
        bbs_pages[i * 4 + 2] = ((uint16_t)i<<8) + (0x2<<4);
        bbs_pages[i * 4 + 3] = ((uint16_t)i<<8) + (0x3<<4);
    }
    bbs_pages[1] = (0x3<<4) + 0x1;

    memmap[1] = (roa_bbs & 0x04 ? ram_page2 : ram_page1);
    LoadRomBank(volume_idx, 0, true);
    memmap[7] = GetRomCacheAddr(volume_idx, 0) + 0x2000;
	memmap[6] = GetAddrFromBbs(roa_bbs & 0x0F);
    SwitchBank();
}

static void GenerateAndPlayJGWav(){

}

static uint8_t* GetPtr40(uint8_t index){
    if (index < 4) {
        return ram_io;
    } else {
        return ram_buff + ((index) << 6);
    }
}

static uint8_t IO_API ReadXX(uint8_t addr){
	return ram_io[addr];
}

static uint8_t IO_API Read06(uint8_t addr){
	return ram_io[addr];
}

static uint8_t IO_API Read3B(uint8_t addr){
    if (!(ram_io[0x3D] & 0x03)) {
        return nc1020.clock_data[0x3B] & 0xFE;
    }
    return ram_io[addr];
}

static uint8_t IO_API Read3F(uint8_t addr){
    uint8_t idx = ram_io[0x3E];
    return idx < 80 ? nc1020.clock_data[idx] : 0;
}

static void IO_API WriteXX(uint8_t addr, uint8_t value){
    ram_io[addr] = value;
}


// switch bank.
static void IO_API Write00(uint8_t addr, uint8_t value){
    uint8_t old_value = ram_io[addr];
    ram_io[addr] = value;
    if (value != old_value) {
    	SwitchBank();
    }
}

static void IO_API Write05(uint8_t addr, uint8_t value){
	uint8_t old_value = ram_io[addr];
	ram_io[addr] = value;
	if ((old_value ^ value) & 0x08) {
		nc1020.slept = !(value & 0x08);
	}
}

static void IO_API Write06(uint8_t addr, uint8_t value){
    ram_io[addr] = value;
    if (!nc1020.lcd_addr) {
    	nc1020.lcd_addr = ((ram_io[0x0C] & 0x03) << 12) | (value << 4);
    }
    ram_io[0x09] &= 0xFE;
}

static void IO_API Write08(uint8_t addr, uint8_t value){
    ram_io[addr] = value;
    ram_io[0x0B] &= 0xFE;
}

// keypad matrix.
static void IO_API Write09(uint8_t addr, uint8_t value){
    ram_io[addr] = value;
    switch (value){
    case 0x01: ram_io[0x08] = nc1020.keypad_matrix[0]; break;
    case 0x02: ram_io[0x08] = nc1020.keypad_matrix[1]; break;
    case 0x04: ram_io[0x08] = nc1020.keypad_matrix[2]; break;
    case 0x08: ram_io[0x08] = nc1020.keypad_matrix[3]; break;
    case 0x10: ram_io[0x08] = nc1020.keypad_matrix[4]; break;
    case 0x20: ram_io[0x08] = nc1020.keypad_matrix[5]; break;
    case 0x40: ram_io[0x08] = nc1020.keypad_matrix[6]; break;
    case 0x80: ram_io[0x08] = nc1020.keypad_matrix[7]; break;
    case 0:
        ram_io[0x0B] |= 1;
        if (nc1020.keypad_matrix[7] == 0xFE) {
            ram_io[0x0B] &= 0xFE;
        }
        break;
    case 0x7F:
        if (ram_io[0x15] == 0x7F) {
            ram_io[0x08] = (
                nc1020.keypad_matrix[0] |
                nc1020.keypad_matrix[1] |
                nc1020.keypad_matrix[2] |
                nc1020.keypad_matrix[3] |
                nc1020.keypad_matrix[4] |
                nc1020.keypad_matrix[5] |
                nc1020.keypad_matrix[6] |
                nc1020.keypad_matrix[7]
            );
        }
        break;
    }
}

// roabbs
static void IO_API Write0A(uint8_t addr, uint8_t value){
    uint8_t old_value = ram_io[addr];
    ram_io[addr] = value;
    if (value != old_value) {
        memmap[6] = GetAddrFromBbs(value & 0x0F);
    }
}

// switch volume
static void IO_API Write0D(uint8_t addr, uint8_t value){
	uint8_t old_value = ram_io[addr];
    ram_io[addr] = value;
    if (value != old_value) {
        SwitchVolume();
    }
}

// zp40 switch
static void IO_API Write0F(uint8_t addr, uint8_t value){
	uint8_t old_value = ram_io[addr];
    ram_io[addr] = value;
    old_value &= 0x07;
    value &= 0x07;
    if (value != old_value) {
        uint8_t* ptr_new = GetPtr40(value);
        if (old_value) {
            memcpy(GetPtr40(old_value), ram_40, 0x40);
            memcpy(ram_40, value ? ptr_new : nc1020.bak_40, 0x40);
        } else {
            memcpy(nc1020.bak_40, ram_40, 0x40);
            memcpy(ram_40, ptr_new, 0x40);
        }
    }
}

static void IO_API Write20(uint8_t addr, uint8_t value){
    ram_io[addr] = value;
    if (value == 0x80 || value == 0x40) {
        memset(nc1020.jg_wav_data, 0, 0x20);
        ram_io[0x20] = 0;
        nc1020.jg_wav_flags = 1;
        nc1020.jg_wav_idx = 0;
    }
}

static void IO_API Write23(uint8_t addr, uint8_t value){
    ram_io[addr] = value;
    if (value == 0xC2) {
        nc1020.jg_wav_data[nc1020.jg_wav_idx] = ram_io[0x22];
    } else if (value == 0xC4) {
        if (nc1020.jg_wav_idx < 0x20) {
            nc1020.jg_wav_data[nc1020.jg_wav_idx] = ram_io[0x22];
            nc1020.jg_wav_idx ++;
        }
    } else if (value == 0x80) {
        ram_io[0x20] = 0x80;
        nc1020.jg_wav_flags = 0;
        if (nc1020.jg_wav_idx) {
            if (!nc1020.jg_wav_playing) {
                GenerateAndPlayJGWav();
                nc1020.jg_wav_idx = 0;
            }
        }
    }
    if (nc1020.jg_wav_playing) {
        // todo.
    }
}

// clock.
static void IO_API Write3F(uint8_t addr, uint8_t value){
    ram_io[addr] = value;
    uint8_t idx = ram_io[0x3E];
    if (idx >= 0x07) {
        if (idx == 0x0B) {
            ram_io[0x3D] = 0xF8;
            nc1020.clock_flags |= value & 0x07;
            nc1020.clock_data[0x0B] = value ^ ((nc1020.clock_data[0x0B] ^ value) & 0x7F);
        } else if (idx == 0x0A) {
            nc1020.clock_flags |= value & 0x07;
            nc1020.clock_data[0x0A] = value;
        } else {
            nc1020.clock_data[idx % 80] = value;
        }
    } else {
        if (!(nc1020.clock_data[0x0B] & 0x80) && idx < 80) {
            nc1020.clock_data[idx] = value;
        }
    }
}

static void AdjustTime(){
    if (++ nc1020.clock_data[0] >= 60) {
        nc1020.clock_data[0] = 0;
        if (++ nc1020.clock_data[1] >= 60) {
            nc1020.clock_data[1] = 0;
            if (++ nc1020.clock_data[2] >= 24) {
                nc1020.clock_data[2] &= 0xC0;
                ++ nc1020.clock_data[3];
            }
        }
    }
}

static bool IsCountDown(){
    if (!(nc1020.clock_data[10] & 0x02) ||
        !(nc1020.clock_flags & 0x02)) {
        return false;
    }
    return (
        ((nc1020.clock_data[7] & 0x80) && !(((nc1020.clock_data[7] ^ nc1020.clock_data[2])) & 0x1F)) ||
        ((nc1020.clock_data[6] & 0x80) && !(((nc1020.clock_data[6] ^ nc1020.clock_data[1])) & 0x3F)) ||
        ((nc1020.clock_data[5] & 0x80) && !(((nc1020.clock_data[5] ^ nc1020.clock_data[0])) & 0x3F))
        );
}

static inline uint8_t Peek(uint16_t addr) {
	if (addr < 0x100)
		return ram_buff[addr];
	else
		return memmap[addr / 0x2000][addr % 0x2000];
}
static inline uint8_t* PeekP(uint16_t addr) {
	if (addr < 0x100)
		return &ram_buff[addr];
	else
		return &memmap[addr / 0x2000][addr % 0x2000];
}
static inline uint16_t PeekW(uint16_t addr) {
	return Peek(addr) | (Peek((uint16_t) (addr + 1)) << 8);
}
static inline uint8_t Load(uint16_t addr) {
	if (addr < IO_LIMIT) {
		return io_read[addr](addr);
	}
	if (((nc1020.fp_step == 4 && nc1020.fp_type == 2) ||
		(nc1020.fp_step == 6 && nc1020.fp_type == 3)) &&
		(addr >= 0x4000 && addr < 0xC000)) {
		nc1020.fp_step = 0;
		return 0x88;
	}
	if (addr == 0x45F && nc1020.pending_wake_up) {
		nc1020.pending_wake_up = false;
		memmap[0][0x45F] = nc1020.wake_up_flags;
	}
	return Peek(addr);
}
static inline void Store(uint16_t addr, uint8_t value) {
	if (addr < IO_LIMIT) {
		io_write[addr](addr, value);
		return;
	}
	if (addr < 0x4000) {
		*PeekP(addr) = value;
		return;
	}
	uint8_t* page = memmap[addr >> 13];
	if (page == ram_page2 || page == ram_page3) {
		page[addr & 0x1FFF] = value;
		return;
	}
	if (addr >= 0xE000) {
		return;
	}

    // write to nor_flash address space.
    // there must select a nor_bank.

    uint8_t bank_idx = ram_io[0x00];
    if (bank_idx >= 0x20) {
        return;
    }

    uint8_t* bank = GetBank(bank_idx);

    if (nc1020.fp_step == 0) {
        if (addr == 0x5555 && value == 0xAA) {
            nc1020.fp_step = 1;
        }
        return;
    }
    if (nc1020.fp_step == 1) {
        if (addr == 0xAAAA && value == 0x55) {
        	nc1020.fp_step = 2;
            return;
        }
    } else if (nc1020.fp_step == 2) {
        if (addr == 0x5555) {
        	switch (value) {
        	case 0x90: nc1020.fp_type = 1; break;
        	case 0xA0: nc1020.fp_type = 2; break;
        	case 0x80: nc1020.fp_type = 3; break;
        	case 0xA8: nc1020.fp_type = 4; break;
        	case 0x88: nc1020.fp_type = 5; break;
        	case 0x78: nc1020.fp_type = 6; break;
        	}
            if (nc1020.fp_type) {
                if (nc1020.fp_type == 1) {
                    nc1020.fp_bank_idx = bank_idx;
                    nc1020.fp_bak1 = bank[0x4000];
                    nc1020.fp_bak1 = bank[0x4001];	// ? bak1?
                }
                nc1020.fp_step = 3;
                return;
            }
        }
    } else if (nc1020.fp_step == 3) {
        if (nc1020.fp_type == 1) {
            if (value == 0xF0) {
                bank[0x4000] = nc1020.fp_bak1;
                bank[0x4001] = nc1020.fp_bak2;
                nc1020.fp_step = 0;
				nor_bank[bank_idx] |= 0x80;	/* dirty */
                return;
            }
        } else if (nc1020.fp_type == 2) {
            bank[addr - 0x4000] &= value;
            nc1020.fp_step = 4;
			nor_bank[bank_idx] |= 0x80;	/* dirty */
            return;
        } else if (nc1020.fp_type == 4) {
            nc1020.fp_buff[addr & 0xFF] &= value;
            nc1020.fp_step = 4;
            return;
        } else if (nc1020.fp_type == 3 || nc1020.fp_type == 5) {
            if (addr == 0x5555 && value == 0xAA) {
                nc1020.fp_step = 4;
                return;
            }
        }
    } else if (nc1020.fp_step == 4) {
        if (nc1020.fp_type == 3 || nc1020.fp_type == 5) {
            if (addr == 0xAAAA && value == 0x55) {
                nc1020.fp_step = 5;
                return;
            }
        }
    } else if (nc1020.fp_step == 5) {
        if (addr == 0x5555 && value == 0x10) {
        	size_t i;
        	for (i=0; i<0x20; i++) {
                memset(GetBank(i), 0xFF, 0x8000);
				nor_bank[i] |= 0x80;	/* dirty */
            }
            if (nc1020.fp_type == 5) {
                memset(nc1020.fp_buff, 0xFF, 0x100);
            }
            nc1020.fp_step = 6;
            return;
        }
        if (nc1020.fp_type == 3) {
            if (value == 0x30) {
                memset(bank + (addr - (addr % 0x800) - 0x4000), 0xFF, 0x800);
                nc1020.fp_step = 6;
				nor_bank[bank_idx] |= 0x80;	/* dirty */
                return;
            }
        } else if (nc1020.fp_type == 5) {
            if (value == 0x48) {
                memset(nc1020.fp_buff, 0xFF, 0x100);
                nc1020.fp_step = 6;
                return;
            }
        }
    }
    if (addr == 0x8000 && value == 0xF0) {
        nc1020.fp_step = 0;
        return;
    }
    printf("error occurs when operate in flash!");
}

static void InitCache() {
    memset(&cache_ctrl, 0, sizeof(cache_ctrl));
    memset(cache_ctrl.rom_priority, 0xFF, sizeof(cache_ctrl.rom_priority));
    memset(cache_ctrl.nor_priority, 0xFF, sizeof(cache_ctrl.nor_priority));

    memset(rom_bank, 0xFF, sizeof(rom_bank));
    memset(nor_bank, 0xFF, sizeof(nor_bank));
}

void wqxInitialize(const char* path) {
	strncpy(nc1020_file_rom, path, sizeof(nc1020_file_rom)-1);
	strncpy(nc1020_file_nor, path, sizeof(nc1020_file_nor)-1);
    strncpy(nc1020_file_nor_tmp, path, sizeof(nc1020_file_nor_tmp)-1);
	strncpy(nc1020_file_sts, path, sizeof(nc1020_file_sts)-1);
	strncat(nc1020_file_rom, "/obj_lu.bin", sizeof(nc1020_file_rom)-strlen(path)-1);
	strncat(nc1020_file_nor, "/nc1020.fls", sizeof(nc1020_file_nor)-strlen(path)-1);
    strncat(nc1020_file_nor_tmp, "/nc1020.fls.tmp", sizeof(nc1020_file_nor_tmp)-strlen(path)-1);
	strncat(nc1020_file_sts, "/nc1020.sts", sizeof(nc1020_file_sts)-strlen(path)-1);

	ram_buff = nc1020.ram;
	stack = ram_buff + 0x100;
	ram_io = ram_buff;
	ram_40 = ram_buff + 0x40;
	ram_page0 = ram_buff;
	ram_page1 = ram_buff + 0x2000;
	ram_page2 = ram_buff + 0x4000;
	ram_page3 = ram_buff + 0x6000;

	size_t i;
	for (i=0; i<0x40; i++) {
		io_read[i] = ReadXX;
		io_write[i] = WriteXX;
	}
	io_read[0x06] = Read06;
	io_read[0x3B] = Read3B;
	io_read[0x3F] = Read3F;
	io_write[0x00] = Write00;
	io_write[0x05] = Write05;
	io_write[0x06] = Write06;
	io_write[0x08] = Write08;
	io_write[0x09] = Write09;
	io_write[0x0A] = Write0A;
	io_write[0x0D] = Write0D;
	io_write[0x0F] = Write0F;
	io_write[0x20] = Write20;
	io_write[0x23] = Write23;
	io_write[0x3F] = Write3F;

	/* init */
	rom_volume[0] = &rom_bank[0 * ROM_VOLUME_BANK_COUNT];
	rom_volume[1] = &rom_bank[1 * ROM_VOLUME_BANK_COUNT];
	rom_volume[2] = &rom_bank[0 * ROM_VOLUME_BANK_COUNT];
	rom_volume[3] = &rom_bank[2 * ROM_VOLUME_BANK_COUNT];

    InitCache();

//#ifdef DEBUG
//	FILE* file = fopen((nc1020_dir + "/wqxsimlogs.bin").c_str(), "rb");
//	fseek(file, 0L, SEEK_END);
//	size_t file_size = ftell(file);
//	size_t insts_count = (file_size - 8) / 8;
//	debug_logs.insts_count = insts_count;
//	debug_logs.logs = (log_rec_t*)malloc(insts_count * 8);
//	fseek(file, 0L, SEEK_SET);
//	fread(&debug_logs.insts_start, 4, 1, file);
//	fseek(file, 4L, SEEK_SET);
//	fread(&debug_logs.peek_addr, 4, 1, file);
//	fseek(file, 8L, SEEK_SET);
//	fread(debug_logs.logs, 8, insts_count, file);
//	fclose(file);
//#endif
}

static void ResetStates(){
	nc1020.version = VERSION;

	memset(ram_buff, 0, 0x8000);
	memmap[0] = ram_page0;
	memmap[2] = ram_page2;
	SwitchVolume();

	memset(nc1020.keypad_matrix, 0, 8);

	memset(nc1020.clock_data, 0, 80);
	nc1020.clock_flags = 0;

	nc1020.timer0_toggle = false;

	memset(nc1020.jg_wav_data, 0, 0x20);
	nc1020.jg_wav_flags = 0;
	nc1020.jg_wav_idx = 0;

	nc1020.should_wake_up = false;
	nc1020.pending_wake_up = false;

	memset(nc1020.fp_buff, 0, 0x100);
	nc1020.fp_step = 0;

	nc1020.should_irq = false;

	nc1020.cycles = 0;
	nc1020.cpu.reg_a = 0;
	nc1020.cpu.reg_ps = 0x24;
	nc1020.cpu.reg_x = 0;
	nc1020.cpu.reg_y = 0;
	nc1020.cpu.reg_sp = 0xFF;
	nc1020.cpu.reg_pc = PeekW(RESET_VEC);
	nc1020.timer0_cycles = CYCLES_TIMER0;
	nc1020.timer1_cycles = CYCLES_TIMER1;

//#ifdef DEBUG
//	executed_insts = 0;
//	debug_done = false;
//#endif
}

void wqxReset() {
    SaveNor();
	if (f_rom)
        fclose(f_rom);
    if (f_nor)
        fclose(f_nor);
    f_rom = fopen(nc1020_file_rom, "rb");
    f_nor = fopen(nc1020_file_nor, "rb");
	ResetStates();
}

static void LoadStates(){
	ResetStates();
	FILE* file = fopen(nc1020_file_sts, "rb");
	if (file == NULL) {
		return;
	}
	fread(&nc1020, 1, sizeof(nc1020), file);
	fclose(file);
	if (nc1020.version != VERSION) {
		return;
	}
	SwitchVolume();
}

static void SaveStates(){
	FILE* file = fopen(nc1020_file_sts, "wb");
	fwrite(&nc1020, 1, sizeof(nc1020), file);
	fflush(file);
	fclose(file);
}

void wqxLoadNC1020(){
    if (!f_rom)
	    f_rom = fopen(nc1020_file_rom, "rb");
    if (!f_nor)
	    f_nor = fopen(nc1020_file_nor, "rb");
	LoadStates();
}

void wqxQuitNC1020(){
    if (f_rom) {
        fclose(f_rom);
        f_rom = NULL;
    }
    if (f_nor) {
        fclose(f_nor);
        f_nor = NULL;
    }
}

void wqxSaveNC1020(){
	SaveNor();
	SaveStates();
}

void wqxSetKey(uint8_t key_id, bool down_or_up){
	uint8_t row = key_id % 8;
	uint8_t col = key_id / 8;
	uint8_t bits = 1 << col;
	if (key_id == 0x0F) {
		bits = 0xFE;
	}
	if (down_or_up) {
		nc1020.keypad_matrix[row] |= bits;
	} else {
		nc1020.keypad_matrix[row] &= ~bits;
	}

	if (down_or_up) {
		if (nc1020.slept) {
			if (key_id >= 0x08 && key_id <= 0x0F && key_id != 0x0E) {
				switch (key_id) {
				case 0x08: nc1020.wake_up_flags = 0x00; break;
				case 0x09: nc1020.wake_up_flags = 0x0A; break;
				case 0x0A: nc1020.wake_up_flags = 0x08; break;
				case 0x0B: nc1020.wake_up_flags = 0x06; break;
				case 0x0C: nc1020.wake_up_flags = 0x04; break;
				case 0x0D: nc1020.wake_up_flags = 0x02; break;
				case 0x0E: nc1020.wake_up_flags = 0x0C; break;
				case 0x0F: nc1020.wake_up_flags = 0x00; break;
				}
				nc1020.should_wake_up = true;
				nc1020.pending_wake_up = true;
				nc1020.slept = false;
			}
		} else {
			if (key_id == 0x0F) {
				nc1020.slept = true;
			}
		}
	}
}

bool wqxCopyLcdBuffer(uint8_t* buffer){
	if (nc1020.lcd_addr == 0) return false;
	memcpy(buffer, ram_buff + nc1020.lcd_addr, 1600);
	return true;
}

void wqxRunTimeSlice(float time_slice, bool speed_up) {
	size_t end_cycles = (size_t)(time_slice * CYCLES_MS);
	register size_t cycles = nc1020.cycles;
	register uint16_t reg_pc = nc1020.cpu.reg_pc;
	register uint8_t reg_a = nc1020.cpu.reg_a;
	register uint8_t reg_ps = nc1020.cpu.reg_ps;
	register uint8_t reg_x = nc1020.cpu.reg_x;
	register uint8_t reg_y = nc1020.cpu.reg_y;
	register uint8_t reg_sp = nc1020.cpu.reg_sp;

	while (cycles < end_cycles) {
//#ifdef DEBUG
//		if (executed_insts == 2792170) {
//			printf("debug start!\n");
//		}
//		if (executed_insts >= debug_logs.insts_start &&
//			executed_insts < debug_logs.insts_start + debug_logs.insts_count) {
//			log_rec_t& log = debug_logs.logs[executed_insts - debug_logs.insts_start];
//			string debug_info;
//			if (log.reg_pc != reg_pc) {
//				debug_info += " pc ";
//			}
//			if (log.reg_a != reg_a) {
//				debug_info += " a ";
//			}
//			if (log.reg_ps != reg_ps) {
//				debug_info += " ps ";
//			}
//			if (log.reg_x != reg_x) {
//				debug_info += " x ";
//			}
//			if (log.reg_y != reg_y) {
//				debug_info += " y ";
//			}
//			if (log.reg_sp != reg_sp) {
//				debug_info += " sp ";
//			}
//			if (debug_logs.peek_addr != -1) {
//				if (log.peeked != Peek((uint16_t)debug_logs.peek_addr)) {
//					debug_info += " mem ";
//				}
//			} else {
//				if (log.peeked != Peek(reg_pc)) {
//					debug_info += " op ";
//				}
//			}
//			if (debug_info.length()) {
//				printf("%d: %s\n", executed_insts, debug_info.c_str());
//				exit(-1);
//			}
//		}
//		if (executed_insts >= debug_logs.insts_start + debug_logs.insts_count) {
//			printf("ok\n");
//		}
//#endif
		switch (Peek(reg_pc++)) {
		case 0x00: {
			reg_pc++;
			stack[reg_sp--] = reg_pc >> 8;
			stack[reg_sp--] = reg_pc & 0xFF;
			reg_ps |= 0x10;
			stack[reg_sp--] = reg_ps;
			reg_ps |= 0x04;
			reg_pc = PeekW(IRQ_VEC);
			cycles += 7;
		}
			break;
		case 0x01: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 6;
		}
			break;
		case 0x02: {
		}
			break;
		case 0x03: {
		}
			break;
		case 0x04: {
		}
			break;
		case 0x05: {
			uint16_t addr = Peek(reg_pc++);
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 3;
		}
			break;
		case 0x06: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 >> 7);
			tmp1 <<= 1;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 5;
		}
			break;
		case 0x07: {
		}
			break;
		case 0x08: {
			stack[reg_sp--] = reg_ps;
			cycles += 3;
		}
			break;
		case 0x09: {
			uint16_t addr = reg_pc++;
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0x0A: {
			reg_ps &= 0x7C;
			reg_ps |= reg_a >> 7;
			reg_a <<= 1;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0x0B: {
		}
			break;
		case 0x0C: {
		}
			break;
		case 0x0D: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x0E: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 >> 7);
			tmp1 <<= 1;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 6;
		}
			break;
		case 0x0F: {
		}
			break;
		case 0x10: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if (!(reg_ps & 0x80)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0x11: {
			uint16_t addr = PeekW(Peek(reg_pc));
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc++;
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 5;
		}
			break;
		case 0x12: {
		}
			break;
		case 0x13: {
		}
			break;
		case 0x14: {
		}
			break;
		case 0x15: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x16: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 >> 7);
			tmp1 <<= 1;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 6;
		}
			break;
		case 0x17: {
		}
			break;
		case 0x18: {
			reg_ps &= 0xFE;
			cycles += 2;
		}
			break;
		case 0x19: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x1A: {
		}
			break;
		case 0x1B: {
		}
			break;
		case 0x1C: {
		}
			break;
		case 0x1D: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			reg_a |= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x1E: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 >> 7);
			tmp1 <<= 1;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 6;
		}
			break;
		case 0x1F: {
		}
			break;
		case 0x20: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_pc--;
			stack[reg_sp--] = reg_pc >> 8;
			stack[reg_sp--] = reg_pc & 0xFF;
			reg_pc = addr;
			cycles += 6;
		}
			break;
		case 0x21: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 6;
		}
			break;
		case 0x22: {
		}
			break;
		case 0x23: {
		}
			break;
		case 0x24: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x3D;
			reg_ps |= (!(reg_a & tmp1) << 1) | (tmp1 & 0xC0);
			cycles += 3;
		}
			break;
		case 0x25: {
			uint16_t addr = Peek(reg_pc++);
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 3;
		}
			break;
		case 0x26: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 << 1) | (reg_ps & 0x01);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >> 7);
			Store(addr, tmp2);
			cycles += 5;
		}
			break;
		case 0x27: {
		}
			break;
		case 0x28: {
			reg_ps = stack[++reg_sp];
			cycles += 4;
		}
			break;
		case 0x29: {
			uint16_t addr = reg_pc++;
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0x2A: {
			uint8_t tmp1 = reg_a;
			reg_a = (reg_a << 1) | (reg_ps & 0x01);
			reg_ps &= 0x7C;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1) | (tmp1 >> 7);
			cycles += 2;
		}
			break;
		case 0x2B: {
		}
			break;
		case 0x2C: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x3D;
			reg_ps |= (!(reg_a & tmp1) << 1) | (tmp1 & 0xC0);
			cycles += 4;
		}
			break;
		case 0x2D: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x2E: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 << 1) | (reg_ps & 0x01);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >> 7);
			Store(addr, tmp2);
			cycles += 6;
		}
			break;
		case 0x2F: {
		}
			break;
		case 0x30: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if ((reg_ps & 0x80)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0x31: {
			uint16_t addr = PeekW(Peek(reg_pc));
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc++;
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 5;
		}
			break;
		case 0x32: {
		}
			break;
		case 0x33: {
		}
			break;
		case 0x34: {
		}
			break;
		case 0x35: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x36: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 << 1) | (reg_ps & 0x01);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >> 7);
			Store(addr, tmp2);
			cycles += 6;
		}
			break;
		case 0x37: {
		}
			break;
		case 0x38: {
			reg_ps |= 0x01;
			cycles += 2;
		}
			break;
		case 0x39: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x3A: {
		}
			break;
		case 0x3B: {
		}
			break;
		case 0x3C: {
		}
			break;
		case 0x3D: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			reg_a &= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x3E: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 << 1) | (reg_ps & 0x01);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >> 7);
			Store(addr, tmp2);
			cycles += 6;
		}
			break;
		case 0x3F: {
		}
			break;
		case 0x40: {
			reg_ps = stack[++reg_sp];
			reg_pc = stack[++reg_sp];
			reg_pc |= stack[++reg_sp] << 8;
			cycles += 6;
		}
			break;
		case 0x41: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 6;
		}
			break;
		case 0x42: {
		}
			break;
		case 0x43: {
		}
			break;
		case 0x44: {
		}
			break;
		case 0x45: {
			uint16_t addr = Peek(reg_pc++);
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 3;
		}
			break;
		case 0x46: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 & 0x01);
			tmp1 >>= 1;
			reg_ps |= (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 5;
		}
			break;
		case 0x47: {
		}
			break;
		case 0x48: {
			stack[reg_sp--] = reg_a;
			cycles += 3;
		}
			break;
		case 0x49: {
			uint16_t addr = reg_pc++;
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0x4A: {
			reg_ps &= 0x7C;
			reg_ps |= reg_a & 0x01;
			reg_a >>= 1;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0x4B: {
		}
			break;
		case 0x4C: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_pc = addr;
			cycles += 3;
		}
			break;
		case 0x4D: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x4E: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 & 0x01);
			tmp1 >>= 1;
			reg_ps |= (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 6;
		}
			break;
		case 0x4F: {
		}
			break;
		case 0x50: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if (!(reg_ps & 0x40)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0x51: {
			uint16_t addr = PeekW(Peek(reg_pc));
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc++;
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 5;
		}
			break;
		case 0x52: {
		}
			break;
		case 0x53: {
		}
			break;
		case 0x54: {
		}
			break;
		case 0x55: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x56: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 & 0x01);
			tmp1 >>= 1;
			reg_ps |= (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 6;
		}
			break;
		case 0x57: {
		}
			break;
		case 0x58: {
			reg_ps &= 0xFB;
			cycles += 2;
		}
			break;
		case 0x59: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x5A: {
		}
			break;
		case 0x5B: {
		}
			break;
		case 0x5C: {
		}
			break;
		case 0x5D: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			reg_a ^= Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x5E: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			reg_ps &= 0x7C;
			reg_ps |= (tmp1 & 0x01);
			tmp1 >>= 1;
			reg_ps |= (!tmp1 << 1);
			Store(addr, tmp1);
			cycles += 6;
		}
			break;
		case 0x5F: {
		}
			break;
		case 0x60: {
			reg_pc = stack[++reg_sp];
			reg_pc |= (stack[++reg_sp] << 8);
			reg_pc++;
			cycles += 6;
		}
			break;
		case 0x61: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 6;
		}
			break;
		case 0x62: {
		}
			break;
		case 0x63: {
		}
			break;
		case 0x64: {
		}
			break;
		case 0x65: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 3;
		}
			break;
		case 0x66: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 >> 1) | ((reg_ps & 0x01) << 7);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 & 0x01);
			Store(addr, tmp2);
			cycles += 5;
		}
			break;
		case 0x67: {
		}
			break;
		case 0x68: {
			reg_a = stack[++reg_sp];
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0x69: {
			uint16_t addr = reg_pc++;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 2;
		}
			break;
		case 0x6A: {
			uint8_t tmp1 = reg_a;
			reg_a = (reg_a >> 1) | ((reg_ps & 0x01) << 7);
			reg_ps &= 0x7C;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1) | (tmp1 & 0x01);
			cycles += 2;
		}
			break;
		case 0x6B: {
		}
			break;
		case 0x6C: {
			uint16_t addr = PeekW(PeekW(reg_pc));
			reg_pc += 2;
			reg_pc = addr;
			cycles += 6;
		}
			break;
		case 0x6D: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0x6E: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 >> 1) | ((reg_ps & 0x01) << 7);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 & 0x01);
			Store(addr, tmp2);
			cycles += 6;
		}
			break;
		case 0x6F: {
		}
			break;
		case 0x70: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if ((reg_ps & 0x40)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0x71: {
			uint16_t addr = PeekW(Peek(reg_pc));
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc++;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 5;
		}
			break;
		case 0x72: {
		}
			break;
		case 0x73: {
		}
			break;
		case 0x74: {
		}
			break;
		case 0x75: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0x76: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 >> 1) | ((reg_ps & 0x01) << 7);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 & 0x01);
			Store(addr, tmp2);
			cycles += 6;
		}
			break;
		case 0x77: {
		}
			break;
		case 0x78: {
			reg_ps |= 0x04;
			cycles += 2;
		}
			break;
		case 0x79: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0x7A: {
		}
			break;
		case 0x7B: {
		}
			break;
		case 0x7C: {
		}
			break;
		case 0x7D: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a + tmp1 + (reg_ps & 0x01);
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 > 0xFF)
					| (((reg_a ^ tmp1 ^ 0x80) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0x7E: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			uint8_t tmp2 = (tmp1 >> 1) | ((reg_ps & 0x01) << 7);
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 & 0x01);
			Store(addr, tmp2);
			cycles += 6;
		}
			break;
		case 0x7F: {
		}
			break;
		case 0x80: {
		}
			break;
		case 0x81: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			Store(addr, reg_a);
			cycles += 6;
		}
			break;
		case 0x82: {
		}
			break;
		case 0x83: {
		}
			break;
		case 0x84: {
			uint16_t addr = Peek(reg_pc++);
			Store(addr, reg_y);
			cycles += 3;
		}
			break;
		case 0x85: {
			uint16_t addr = Peek(reg_pc++);
			Store(addr, reg_a);
			cycles += 3;
		}
			break;
		case 0x86: {
			uint16_t addr = Peek(reg_pc++);
			Store(addr, reg_x);
			cycles += 3;
		}
			break;
		case 0x87: {
		}
			break;
		case 0x88: {
			reg_y--;
			reg_ps &= 0x7D;
			reg_ps |= (reg_y & 0x80) | (!reg_y << 1);
			cycles += 2;
		}
			break;
		case 0x89: {
		}
			break;
		case 0x8A: {
			reg_a = reg_x;
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0x8B: {
		}
			break;
		case 0x8C: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			Store(addr, reg_y);
			cycles += 4;
		}
			break;
		case 0x8D: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			Store(addr, reg_a);
			cycles += 4;
		}
			break;
		case 0x8E: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			Store(addr, reg_x);
			cycles += 4;
		}
			break;
		case 0x8F: {
		}
			break;
		case 0x90: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if (!(reg_ps & 0x01)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0x91: {
			uint16_t addr = PeekW(Peek(reg_pc));
			addr += reg_y;
			reg_pc++;
			Store(addr, reg_a);
			cycles += 6;
		}
			break;
		case 0x92: {
		}
			break;
		case 0x93: {
		}
			break;
		case 0x94: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			Store(addr, reg_y);
			cycles += 4;
		}
			break;
		case 0x95: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			Store(addr, reg_a);
			cycles += 4;
		}
			break;
		case 0x96: {
			uint16_t addr = (Peek(reg_pc++) + reg_y) & 0xFF;
			Store(addr, reg_x);
			cycles += 4;
		}
			break;
		case 0x97: {
		}
			break;
		case 0x98: {
			reg_a = reg_y;
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0x99: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_y;
			reg_pc += 2;
			Store(addr, reg_a);
			cycles += 5;
		}
			break;
		case 0x9A: {
			reg_sp = reg_x;
			cycles += 2;
		}
			break;
		case 0x9B: {
		}
			break;
		case 0x9C: {
		}
			break;
		case 0x9D: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_x;
			reg_pc += 2;
			Store(addr, reg_a);
			cycles += 5;
		}
			break;
		case 0x9E: {
		}
			break;
		case 0x9F: {
		}
			break;
		case 0xA0: {
			uint16_t addr = reg_pc++;
			reg_y = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_y & 0x80) | (!reg_y << 1);
			cycles += 2;
		}
			break;
		case 0xA1: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 6;
		}
			break;
		case 0xA2: {
			uint16_t addr = reg_pc++;
			reg_x = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 2;
		}
			break;
		case 0xA3: {
		}
			break;
		case 0xA4: {
			uint16_t addr = Peek(reg_pc++);
			reg_y = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_y & 0x80) | (!reg_y << 1);
			cycles += 3;
		}
			break;
		case 0xA5: {
			uint16_t addr = Peek(reg_pc++);
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 3;
		}
			break;
		case 0xA6: {
			uint16_t addr = Peek(reg_pc++);
			reg_x = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 3;
		}
			break;
		case 0xA7: {
		}
			break;
		case 0xA8: {
			reg_y = reg_a;
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0xA9: {
			uint16_t addr = reg_pc++;
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0xAA: {
			reg_x = reg_a;
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 2;
		}
			break;
		case 0xAB: {
		}
			break;
		case 0xAC: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_y = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_y & 0x80) | (!reg_y << 1);
			cycles += 4;
		}
			break;
		case 0xAD: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0xAE: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			reg_x = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 4;
		}
			break;
		case 0xAF: {
		}
			break;
		case 0xB0: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if ((reg_ps & 0x01)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0xB1: {
			uint16_t addr = PeekW(Peek(reg_pc));
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc++;
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 5;
		}
			break;
		case 0xB2: {
		}
			break;
		case 0xB3: {
		}
			break;
		case 0xB4: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			reg_y = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_y & 0x80) | (!reg_y << 1);
			cycles += 4;
		}
			break;
		case 0xB5: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0xB6: {
			uint16_t addr = (Peek(reg_pc++) + reg_y) & 0xFF;
			reg_x = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 4;
		}
			break;
		case 0xB7: {
		}
			break;
		case 0xB8: {
			reg_ps &= 0xBF;
			cycles += 2;
		}
			break;
		case 0xB9: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0xBA: {
			reg_x = reg_sp;
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 2;
		}
			break;
		case 0xBB: {
		}
			break;
		case 0xBC: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			reg_y = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_y & 0x80) | (!reg_y << 1);
			cycles += 4;
		}
			break;
		case 0xBD: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			reg_a = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_a & 0x80) | (!reg_a << 1);
			cycles += 4;
		}
			break;
		case 0xBE: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			reg_x = Load(addr);
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 4;
		}
			break;
		case 0xBF: {
		}
			break;
		case 0xC0: {
			uint16_t addr = reg_pc++;
			int16_t tmp1 = reg_y - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 2;
		}
			break;
		case 0xC1: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 6;
		}
			break;
		case 0xC2: {
		}
			break;
		case 0xC3: {
		}
			break;
		case 0xC4: {
			uint16_t addr = Peek(reg_pc++);
			int16_t tmp1 = reg_y - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 3;
		}
			break;
		case 0xC5: {
			uint16_t addr = Peek(reg_pc++);
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 3;
		}
			break;
		case 0xC6: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr) - 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 5;
		}
			break;
		case 0xC7: {
		}
			break;
		case 0xC8: {
			reg_y++;
			reg_ps &= 0x7D;
			reg_ps |= (reg_y & 0x80) | (!reg_y << 1);
			cycles += 2;
		}
			break;
		case 0xC9: {
			uint16_t addr = reg_pc++;
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 2;
		}
			break;
		case 0xCA: {
			reg_x--;
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 2;
		}
			break;
		case 0xCB: {
		}
			break;
		case 0xCC: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			int16_t tmp1 = reg_y - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 4;
		}
			break;
		case 0xCD: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 4;
		}
			break;
		case 0xCE: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr) - 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 6;
		}
			break;
		case 0xCF: {
		}
			break;
		case 0xD0: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if (!(reg_ps & 0x02)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0xD1: {
			uint16_t addr = PeekW(Peek(reg_pc));
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc++;
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 5;
		}
			break;
		case 0xD2: {
		}
			break;
		case 0xD3: {
		}
			break;
		case 0xD4: {
		}
			break;
		case 0xD5: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 4;
		}
			break;
		case 0xD6: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr) - 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 6;
		}
			break;
		case 0xD7: {
		}
			break;
		case 0xD8: {
			reg_ps &= 0xF7;
			cycles += 2;
		}
			break;
		case 0xD9: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 4;
		}
			break;
		case 0xDA: {
		}
			break;
		case 0xDB: {
		}
			break;
		case 0xDC: {
		}
			break;
		case 0xDD: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			int16_t tmp1 = reg_a - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 4;
		}
			break;
		case 0xDE: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr) - 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 6;
		}
			break;
		case 0xDF: {
		}
			break;
		case 0xE0: {
			uint16_t addr = reg_pc++;
			int16_t tmp1 = reg_x - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 2;
		}
			break;
		case 0xE1: {
			uint16_t addr = PeekW((Peek(reg_pc++) + reg_x) & 0xFF);
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 6;
		}
			break;
		case 0xE2: {
		}
			break;
		case 0xE3: {
		}
			break;
		case 0xE4: {
			uint16_t addr = Peek(reg_pc++);
			int16_t tmp1 = reg_x - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 3;
		}
			break;
		case 0xE5: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 3;
		}
			break;
		case 0xE6: {
			uint16_t addr = Peek(reg_pc++);
			uint8_t tmp1 = Load(addr) + 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 5;
		}
			break;
		case 0xE7: {
		}
			break;
		case 0xE8: {
			reg_x++;
			reg_ps &= 0x7D;
			reg_ps |= (reg_x & 0x80) | (!reg_x << 1);
			cycles += 2;
		}
			break;
		case 0xE9: {
			uint16_t addr = reg_pc++;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 2;
		}
			break;
		case 0xEA: {
			cycles += 2;
		}
			break;
		case 0xEB: {
		}
			break;
		case 0xEC: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			int16_t tmp1 = reg_x - Load(addr);
			uint8_t tmp2 = tmp1 & 0xFF;
			reg_ps &= 0x7C;
			reg_ps |= (tmp2 & 0x80) | (!tmp2 << 1) | (tmp1 >= 0);
			cycles += 4;
		}
			break;
		case 0xED: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0xEE: {
			uint16_t addr = PeekW(reg_pc);
			reg_pc += 2;
			uint8_t tmp1 = Load(addr) + 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 6;
		}
			break;
		case 0xEF: {
		}
			break;
		case 0xF0: {
			int8_t tmp4 = (int8_t) (Peek(reg_pc++));
			uint16_t addr = reg_pc + tmp4;
			if ((reg_ps & 0x02)) {
				cycles += !((reg_pc ^ addr) & 0xFF00) << 1;
				reg_pc = addr;
			}
			cycles += 2;
		}
			break;
		case 0xF1: {
			uint16_t addr = PeekW(Peek(reg_pc));
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc++;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 5;
		}
			break;
		case 0xF2: {
		}
			break;
		case 0xF3: {
		}
			break;
		case 0xF4: {
		}
			break;
		case 0xF5: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0xF6: {
			uint16_t addr = (Peek(reg_pc++) + reg_x) & 0xFF;
			uint8_t tmp1 = Load(addr) + 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 6;
		}
			break;
		case 0xF7: {
		}
			break;
		case 0xF8: {
			reg_ps |= 0x08;
			cycles += 2;
		}
			break;
		case 0xF9: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_y) & 0xFF00);
			addr += reg_y;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0xFA: {
		}
			break;
		case 0xFB: {
		}
			break;
		case 0xFC: {
		}
			break;
		case 0xFD: {
			uint16_t addr = PeekW(reg_pc);
			cycles += !!(((addr & 0xFF) + reg_x) & 0xFF00);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr);
			int16_t tmp2 = reg_a - tmp1 + (reg_ps & 0x01) - 1;
			uint8_t tmp3 = tmp2 & 0xFF;
			reg_ps &= 0x3C;
			reg_ps |= (tmp3 & 0x80) | (!tmp3 << 1) | (tmp2 >= 0)
					| (((reg_a ^ tmp1) & (reg_a ^ tmp3) & 0x80) >> 1);
			reg_a = tmp3;
			cycles += 4;
		}
			break;
		case 0xFE: {
			uint16_t addr = PeekW(reg_pc);
			addr += reg_x;
			reg_pc += 2;
			uint8_t tmp1 = Load(addr) + 1;
			Store(addr, tmp1);
			reg_ps &= 0x7D;
			reg_ps |= (tmp1 & 0x80) | (!tmp1 << 1);
			cycles += 6;
		}
			break;
		case 0xFF: {
		}
			break;
		}
//#ifdef DEBUG
//		if (should_irq && !(reg_ps & 0x04)) {
//			should_irq = false;
//			stack[reg_sp --] = reg_pc >> 8;
//			stack[reg_sp --] = reg_pc & 0xFF;
//			reg_ps &= 0xEF;
//			stack[reg_sp --] = reg_ps;
//			reg_pc = PeekW(IRQ_VEC);
//			reg_ps |= 0x04;
//			cycles += 7;
//		}
//		executed_insts ++;
//		if (executed_insts % 6000 == 0) {
//			timer1_cycles += CYCLES_TIMER1;
//			nc1020.clock_data[4] ++;
//			if (should_wake_up) {
//				should_wake_up = false;
//				ram_io[0x01] |= 0x01;
//				ram_io[0x02] |= 0x01;
//				reg_pc = PeekW(RESET_VEC);
//			} else {
//				ram_io[0x01] |= 0x08;
//				should_irq = true;
//			}
//		}
//#else
		if (cycles >= nc1020.timer0_cycles) {
			nc1020.timer0_cycles += CYCLES_TIMER0;
			nc1020.timer0_toggle = !nc1020.timer0_toggle;
			if (!nc1020.timer0_toggle) {
				AdjustTime();
			}
			if (!IsCountDown() || nc1020.timer0_toggle) {
				ram_io[0x3D] = 0;
			} else {
				ram_io[0x3D] = 0x20;
				nc1020.clock_flags &= 0xFD;
			}
			nc1020.should_irq = true;
		}
		if (nc1020.should_irq && !(reg_ps & 0x04)) {
			nc1020.should_irq = false;
			stack[reg_sp --] = reg_pc >> 8;
			stack[reg_sp --] = reg_pc & 0xFF;
			reg_ps &= 0xEF;
			stack[reg_sp --] = reg_ps;
			reg_pc = PeekW(IRQ_VEC);
			reg_ps |= 0x04;
			cycles += 7;
		}
		if (cycles >= nc1020.timer1_cycles) {
			if (speed_up) {
				nc1020.timer1_cycles += CYCLES_TIMER1_SPEED_UP;
			} else {
				nc1020.timer1_cycles += CYCLES_TIMER1;
			}
			nc1020.clock_data[4] ++;
			if (nc1020.should_wake_up) {
				nc1020.should_wake_up = false;
				ram_io[0x01] |= 0x01;
				ram_io[0x02] |= 0x01;
				reg_pc = PeekW(RESET_VEC);
			} else {
				ram_io[0x01] |= 0x08;
				nc1020.should_irq = true;
			}
		}
//#endif
	}

	cycles -= end_cycles;
	nc1020.timer0_cycles -= end_cycles;
	nc1020.timer1_cycles -= end_cycles;

	nc1020.cpu.reg_pc = reg_pc;
	nc1020.cpu.reg_a = reg_a;
	nc1020.cpu.reg_ps = reg_ps;
	nc1020.cpu.reg_x = reg_x;
	nc1020.cpu.reg_y = reg_y;
	nc1020.cpu.reg_sp = reg_sp;
}


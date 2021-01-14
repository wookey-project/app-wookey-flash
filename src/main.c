/**
 * @file main.c
 *
 * \brief Main of dummy
 *
 */

#include "libc/syscall.h"
#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "libc/random.h"
#include "wookey_ipc.h"
#include "libfw.h"
#include "main.h"
#include "automaton.h"
#include "generated/led1.h"
/* Include the private overencryption key from
 * the private folder.
 */
#include "symmetric_overencrypt_dfu_key_iv.h"
/* Include the AES header */
#include "aes.h"
/* Flash key mapping */
#include "generated/bsram_flashkey.h"

static volatile uint32_t total_read = 0;

/* The flash task drives the blue LED when writing chunks */
static inline void led_on(void)
{
#if CONFIG_WOOKEY
    /* toggle led ON */
    sys_cfg(CFG_GPIO_SET, (uint8_t)((led1_dev_infos.gpios[LED1].port << 4) + led1_dev_infos.gpios[LED1].pin), 1);
#endif
}
static inline void led_off(void)
{
#if CONFIG_WOOKEY
    /* toggle led OFF */
    sys_cfg(CFG_GPIO_SET, (uint8_t)((led1_dev_infos.gpios[LED1].port << 4) + led1_dev_infos.gpios[LED1].pin), 0);
#endif
}


/*
 * We use the local -fno-stack-protector flag for main because
 * the stack protection has not been initialized yet.
 *
 * We use _main and not main to permit the usage of exactly *one* arg
 * without compiler complain. argc/argv is not a goot idea in term
 * of size and calculation in a microcontroler
 */
#define FLASH_BUF_SIZE 4096

/* NOTE: alignment due to DMA */
__attribute__((aligned(4))) uint8_t flash_buf[FLASH_BUF_SIZE] = { 0 };

/* Do we check CRC value of each successive storage block ?
 * The final CRC calculation should be equal to the firmware image global CRC
 * This is a debug helper feature, depending on FLASH_DEBUG.
 * INFO: The CRC calculation is Linux CRC compatible
 */
#define CRC 0

#ifdef CRC
volatile uint32_t crc_value = 0xffffffff;
volatile uint16_t block_num = 0;
#endif

static uint8_t id_dfucrypto;
static uint8_t id_dfusmart;

static uint32_t flash_size = 0;
static physaddr_t addr_base = 0;

/*
 * Let's select which part of the flash device is required by the DFU storage
 * process. We need the other bank and the cfg registers to be mapped.
 */
void init_flash_map(void)
{
    if (is_in_flip_mode()) {
        t_device_mapping devmap = {
#if CONFIG_WOOKEY
            .map_flip_shr = 0,
            .map_flip = 0,
            .map_flop_shr = 0,
            .map_flop = 1,
#else
# if CONFIG_USR_DRV_FLASH_DUAL_BANK
            .map_mem_bank1 = 0,
            .map_mem_bank2 = 1,
# else
            .map_mem = 1,
# endif
#endif
            .map_ctrl = 1,
#if CONFIG_WOOKEY
            .map_ctrl_2 = 0,
#endif
            .map_system = 0,
            .map_otp = 0,
            .map_opt_bank1 = 0,
#if CONFIG_USR_DRV_FLASH_DUAL_BANK
            .map_opt_bank2 = 1,
#endif
        };
        firmware_early_init(&devmap);
        // mapping flop
    } else if (is_in_flop_mode()) {
        // mapping flip
        t_device_mapping devmap = {
#if CONFIG_WOOKEY
            .map_flip_shr = 0,
            .map_flip = 1,
            .map_flop_shr = 0,
            .map_flop = 0,
#else
# if CONFIG_USR_DRV_FLASH_DUAL_BANK
            .map_mem_bank1 = 1,
            .map_mem_bank2 = 0,
# else
            .map_mem = 1,
# endif
#endif
            .map_ctrl = 1,
#if CONFIG_WOOKEY
            .map_ctrl_2 = 0,
#endif
            .map_system = 0,
            .map_otp = 0,
            .map_opt_bank1 = 1,
#if CONFIG_USR_DRV_FLASH_DUAL_BANK
            .map_opt_bank2 = 0,
#endif
        };
        firmware_early_init(&devmap);
    }
}

/* Partition mode detection and flash base address and size computation */
static void get_flash_size(void)
{
    if (is_in_flip_mode()) {
        addr_base = CONFIG_USR_LIB_FIRMWARE_FLOP_ADDR;
        flash_size = firmware_get_flop_size();
    } else if (is_in_flop_mode()) {
        addr_base = CONFIG_USR_LIB_FIRMWARE_FLIP_ADDR;
        flash_size = firmware_get_flip_size();
    } else {
        printf("neither in flip or flop mode !!! leaving !\n");
    }
}

/* Backup SRAM handling to get the flash key */
#ifdef CONFIG_APP_DFUFLASH_USE_BKUP_SRAM
/* Map and unmap the Backup SRAM */
static volatile bool bsram_flashkey_is_mapped = false;
static volatile int  dev_bsram_flashkey_desc = 0;
static int bsram_flashkey_init(void){
    const char *name = "bsram-flashkey";
    e_syscall_ret ret = 0;

    device_t dev;
    memset((void*)&dev, 0, sizeof(device_t));
    strncpy(dev.name, name, sizeof (dev.name));
    dev.address = bsram_flashkey_dev_infos.address;
    dev.size = bsram_flashkey_dev_infos.size;
    dev.map_mode = DEV_MAP_VOLUNTARY;

    dev.irq_num = 0;
    dev.gpio_num = 0;
    int dev_bsram_flashkey_desc_ = dev_bsram_flashkey_desc;
    ret = sys_init(INIT_DEVACCESS, &dev, (int*)&dev_bsram_flashkey_desc_);
    if(ret != SYS_E_DONE){
        printf("Error: Backup SRAM flash key, sys_init error!\n");
        goto err;
    }
    dev_bsram_flashkey_desc = dev_bsram_flashkey_desc_;

    return 0;
err:
    return -1;
}

static int bsram_flashkey_map(void){
    if(bsram_flashkey_is_mapped == false){
        e_syscall_ret ret;
        ret = sys_cfg(CFG_DEV_MAP, dev_bsram_flashkey_desc);
        if (ret != SYS_E_DONE) {
            printf("Unable to map Backup SRAM flash key!\n");
            goto err;
        }
        bsram_flashkey_is_mapped = true;
    }

    return 0;
err:
    return -1;
}

static int bsram_flashkey_unmap(void){
    if(bsram_flashkey_is_mapped){
        e_syscall_ret ret;
        ret = sys_cfg(CFG_DEV_UNMAP, dev_bsram_flashkey_desc);
        if (ret != SYS_E_DONE) {
            printf("Unable to unmap Backup SRAM flash key!\n");
            goto err;
        }
        bsram_flashkey_is_mapped = false;
    }

    return 0;
err:
    return -1;
}
#endif


/* Our AES-CTR key */
static uint8_t aes_ctr_decrypt_key[16] = { 0 };
/* Our AES-CTR IV */
static uint8_t aes_ctr_decrypt_iv[16] = { 0 };
static int decrypt_load_keys(void){
    if(sizeof(symmetric_overencrypt_dfu_key_iv) < 32){
        goto err;
    }

#ifdef CONFIG_APP_DFUFLASH_USE_BKUP_SRAM
    /* Map the Backup SRAM for flash key */
    if(bsram_flashkey_map()){
        goto err;
    }
#endif

    /* Copy our AES-CTR key and IV in local SRAM once and for all */
    memcpy(aes_ctr_decrypt_key, &(symmetric_overencrypt_dfu_key_iv[0]), 16);
    memcpy(aes_ctr_decrypt_iv, &(symmetric_overencrypt_dfu_key_iv[16]), 16);

#ifdef CONFIG_APP_DFUFLASH_USE_BKUP_SRAM
    /* Unmap the Backup SRAM for flash key */
    if(bsram_flashkey_unmap()){
        goto err;
    }
#endif  
    return 0;
err:
#ifdef CONFIG_APP_DFUFLASH_USE_BKUP_SRAM
    /* Unmap the Backup SRAM for flash key */
    if(bsram_flashkey_unmap()){
        goto err;
    }
#endif
    return -1;
}

static volatile aes_context aes_context_flash_ctr;
static int decrypt_init(uint16_t chunk_num){    
    /* Initialize our AES-CTR context */
    uint8_t iv[sizeof(aes_ctr_decrypt_iv)] = { 0 };
    memcpy(iv, aes_ctr_decrypt_iv, sizeof(iv));
    add_iv(iv, chunk_num);
    if(aes_init((aes_context*)&aes_context_flash_ctr, aes_ctr_decrypt_key, AES128, iv, CTR, AES_DECRYPT, AES_SOFT_ANSSI_MASKED, NULL, NULL, -1, -1)){
        printf("Flash AES init error!\n");
        goto err;
    }

    return 0;
err:
    return -1;
}

static void decrypt_cleanup_keys(void){
    memset(aes_ctr_decrypt_key, 0, sizeof(aes_ctr_decrypt_key));
    memset(aes_ctr_decrypt_iv, 0, sizeof(aes_ctr_decrypt_iv));
    memset((void*)&aes_context_flash_ctr, 0, sizeof(aes_context_flash_ctr));
#ifdef CONFIG_APP_DFUFLASH_USE_BKUP_SRAM
    /* In case of BSRAM usage, we override the const qualifier on purpose here! */
    memset((uint8_t*)symmetric_overencrypt_dfu_key_iv, 0, sizeof(symmetric_overencrypt_dfu_key_iv));
#endif
    return;
}

static volatile uint32_t last_chunk_num = 0;
static int decrypt_chunk(uint8_t *chunk, uint32_t chunk_size, uint16_t chunk_num)
{
    /* [RB] NOTE: we limit oracle decryption here.
     */
    if((uint32_t)chunk_num != (last_chunk_num + 1)){
        if(decrypt_init(chunk_num)){
            goto err;
        }
    }
    
    /* [RB] NOTE: we switch random generation to non secure here mainly
     * for *performance* reasons! This should however not have much impact
     * on security, since we still rely on the platform TRNG.
     */
    random_secure = SEC_RANDOM_NONSECURE;
    if(aes_exec((aes_context*)&aes_context_flash_ctr, chunk, chunk, chunk_size, -1, -1)){
        random_secure = SEC_RANDOM_SECURE;
        printf("Flash AES exec error!\n");
        goto err;
    }
    random_secure = SEC_RANDOM_SECURE;

    last_chunk_num = chunk_num;

    return 0;
err:
    return -1;
}


/*
 * This is the main loop function, from which the task should not leave
 */
static void main_loop(void)
{
    uint8_t ret;
    uint32_t buffer_count = 0;
    struct sync_command_data dataplane_command_wr;
    struct sync_command_data dataplane_command_ack = { 0 };
    t_ipc_command ipc_mainloop_cmd;
    volatile physaddr_t addr = 0;
    bool flash_is_mapped = false;
    uint8_t id = id_dfucrypto;

    memset(&ipc_mainloop_cmd, 0, sizeof(t_ipc_command));

    /* Flash decrypt load keys */
    if(decrypt_load_keys()){
        goto bad_transition;
    }
 
    while (1) {
        /* initialize logsize to current binary buffer size */
        logsize_t size = sizeof(struct sync_command_data);

        ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_mainloop_cmd);
        if (ret != SYS_E_DONE) {
            /* This may happen only if DFUCRYPTO is also in receive state.
             * If it is a DENIED return, check the application permissions
            */
            continue;
        }

        /****************************************************
         * let's process what we have received...
         ****************************************************/
        switch (ipc_mainloop_cmd.magic) {

            /* Write command request */
            case MAGIC_DATA_WR_DMA_REQ:
                {
                    /******* Automaton handling *********/

                    /* checking transition */
                    if (is_valid_transition(get_task_state(), (uint8_t)MAGIC_DATA_WR_DMA_REQ) != sectrue) {
                        goto bad_transition;
                    }
                    /* updating task state if needed */
                    uint8_t next_state = get_next_state(get_task_state(), (uint8_t)MAGIC_DATA_WR_DMA_REQ);
                    if (next_state != 0xff) {
                        set_task_state(next_state);
                    }

                    /******** executing transition ******/
                    dataplane_command_wr = ipc_mainloop_cmd.sync_cmd_data;
                    block_num =  (uint16_t)dataplane_command_wr.data.u16[1];
#if FLASH_DEBUG
                    printf("received DMA write command to FLASH: blocknum:%x size: %x\n",
                            (uint16_t)dataplane_command_wr.data.u16[1], (uint16_t)dataplane_command_wr.data.u16[0]);
#endif
#if CRC
                    crc_value = crc32(flash_buf, dataplane_command_wr.data.u16[0], crc_value);
#if FLASH_DEBUG
                    printf("[CRC32] current crc is %x (chunk %x)\n", crc_value ^ 0xffffffff, block_num);
#endif
#endif
                    /* let's write ! */
                    uint32_t bufsize = (uint32_t)dataplane_command_wr.data.u16[0];
                    addr = addr_base + (block_num * FLASH_BUF_SIZE);
#if FLASH_DEBUG
                    printf("Writing flash @%x, firmware block %d\n", addr, block_num);
#endif

                    if(decrypt_chunk(flash_buf, bufsize, block_num)){
                        goto bad_transition;
                    }

                    if (!flash_is_mapped) {
                        if(fw_storage_prepare_access()){
                            goto bad_transition;
                        }
                        flash_is_mapped = true;
                    }
		    led_on();
                    fw_storage_write_buffer(addr, (uint32_t*)flash_buf, bufsize);
		    led_off();

                    /*returning the number of bytes read */
                    dataplane_command_ack.magic = MAGIC_DATA_WR_DMA_ACK;
                    dataplane_command_ack.data.u16[0] = dataplane_command_wr.data.u16[0];
                    dataplane_command_ack.data_size = 1;

                    ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, sizeof(struct sync_command_data), (const char*)&dataplane_command_ack);

                    if (ret != SYS_E_DONE) {
                        printf("Error ! unable to send back DMA_WR_ACK to crypto!\n");
                    }
                    buffer_count++;
                    break;
                }

            /* Read command request */
            case MAGIC_DATA_RD_DMA_REQ:
                {
                    /******* Automaton handling *********/

                    /* checking transition */
                    if (is_valid_transition(get_task_state(), (uint8_t)MAGIC_DATA_RD_DMA_REQ) != sectrue) {
                        goto bad_transition;
                    }
                    /* updating task state if needed */
                    uint8_t next_state = get_next_state(get_task_state(), (uint8_t)MAGIC_DATA_RD_DMA_REQ);
                    if (next_state != 0xff) {
                        set_task_state(next_state);
                    }

                    /******** executing transition ******/
                    uint16_t read_data = 0;
                    uint32_t prev_total = total_read;
                    dataplane_command_wr = ipc_mainloop_cmd.sync_cmd_data;
                    block_num = dataplane_command_wr.data.u16[1];
#if FLASH_DEBUG

                    printf("received DMA read command to FLASH: blocknum:%x size: %x\n",
                            dataplane_command_wr.data.u16[1], dataplane_command_wr.data.u16[0]);
#endif
                    total_read += dataplane_command_wr.data.u16[0];
                    if (total_read > flash_size) {
                        /* reinit for next upload if needed */
                        total_read = 0;
                        read_data = flash_size - prev_total;
#if FLASH_DEBUG
                        printf("end of flash read, residual size is %x\n", read_data);
#endif
                    } else {
                        read_data = dataplane_command_wr.data.u16[0];
                    }
                    /* read request.... by now not implemented as it is a security risk,
                     * we transparently return as if we have read data, without updating the
                     * buffer.
                     * INFO: the libdfu support UPLOAD mode, but dfuflash is keept voluntarily
                     * simple and basic by now */

                    /*returning the number of bytes read */
                    dataplane_command_ack.magic = MAGIC_DATA_RD_DMA_ACK;
                    dataplane_command_ack.data.u16[0] = read_data;
                    dataplane_command_ack.data_size = 1;

                    ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, sizeof(struct sync_command_data), (const char*)&dataplane_command_ack);

                    if (ret != SYS_E_DONE) {
                        printf("Error ! unable to send back DMA_WR_ACK to crypto!\n");
                    }
                    break;
                }

            /* End of flash command request */
            case MAGIC_DFU_DWNLOAD_FINISHED:
                {
                    /******* Automaton handling *********/

                    /* checking transition */
                    if (is_valid_transition(get_task_state(), (uint8_t)MAGIC_DFU_DWNLOAD_FINISHED) != sectrue) {
                        goto bad_transition;
                    }
                    /* updating task state if needed */
                    uint8_t next_state = get_next_state(get_task_state(), (uint8_t)MAGIC_DFU_DWNLOAD_FINISHED);
                    if (next_state != 0xff) {
                        set_task_state(next_state);
                    }

                    /******** executing transition ******/

#if CRYPTO_DEBUG
                    printf("receiving EOF from DFUUSB\n");
#endif

                    if (!flash_is_mapped) {
                        if(fw_storage_prepare_access()){
                            goto bad_transition;
                        }
                        flash_is_mapped = true;
                    }
                    /* Here, we voluntary unmap and loose the ability to map the flash (device is released) */
                    fw_storage_finalize_access();

                    /* Then we acknowledge */
                    dataplane_command_ack.magic = MAGIC_DFU_WRITE_FINISHED;
                    dataplane_command_ack.state = SYNC_DONE;

                    ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, sizeof(struct sync_command), (const char*)&dataplane_command_ack);

                    if (ret != SYS_E_DONE) {
                        printf("Error ! unable to send back DFU_WRITE_FINISHED to crypto!\n");
                    }
                    break;
                }

            /*  Unsupported command received */
            default:
                {
                    set_task_state(DFUFLASH_STATE_ERROR);

                    /******** executing transition ******/

                    printf("received invalid command from CRYPTO (magic: %d\n", ipc_mainloop_cmd.magic);
                    ipc_mainloop_cmd.magic = MAGIC_INVALID;
                    ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, sizeof(t_ipc_command), (const char*)&ipc_mainloop_cmd);
                    if(ret != SYS_E_DONE){
                        printf("Error ! unable to send back DFUFLASH_STATE_ERROR to crypto!\n");
                    }
                    break;

                }
        }
    }
    /* bad transition case: leaving the main loop with error */
bad_transition:
    /* Erase our decrypt keys */
    decrypt_cleanup_keys();
    printf("invalid transition from state %d, magic %x\n", get_task_state(),
            ipc_mainloop_cmd.magic);
    set_task_state(DFUFLASH_STATE_ERROR);
    return;
}

int _main(uint32_t task_id)
{
    e_syscall_ret ret;
    char *wellcome_msg = "hello, I'm flash";
    struct sync_command ipc_sync_cmd;
    uint8_t id;
    dma_shm_t dmashm_rd;
    dma_shm_t dmashm_wr;

    dma_shm_t dmashm_flash;

    set_task_state(DFUFLASH_STATE_INIT);

    printf("%s, my id is %x\n", wellcome_msg, task_id);

    ret = sys_init(INIT_GETTASKID, "dfucrypto", &id_dfucrypto);
    printf("dfucrypto is task %x !\n", id_dfucrypto);
    ret = sys_init(INIT_GETTASKID, "dfusmart", &id_dfusmart);
    printf("dfusmart is task %x !\n", id_dfusmart);

    /*********************************************
     * Declaring flash read/write access LED
     ********************************************/
#if CONFIG_WOOKEY
    // led info
    //
    int led_desc;
    device_t led_dev;
    printf("Declaring flash backend LED device\n");
    memset(&led_dev, 0, sizeof(device_t));
    strncpy(led_dev.name, "flash_led", sizeof("flash_led"));
    led_dev.gpio_num = 1;
    led_dev.gpios[0].mask = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_SPEED;
    led_dev.gpios[0].kref.port = led1_dev_infos.gpios[LED1].port;
    led_dev.gpios[0].kref.pin = led1_dev_infos.gpios[LED1].pin;
    led_dev.gpios[0].pupd = GPIO_NOPULL;
    led_dev.gpios[0].mode = GPIO_PIN_OUTPUT_MODE;
    led_dev.gpios[0].speed = GPIO_PIN_HIGH_SPEED;

    ret = sys_init(INIT_DEVACCESS, &led_dev, &led_desc);
    if (ret != 0) {
        printf("Error while declaring LED GPIO device: %d\n", ret);
    }
#endif

    /*********************************************
     * Declaring DMA Shared Memory with Crypto
     *********************************************/
    dmashm_rd.target = id_dfucrypto;
    dmashm_rd.source = task_id;
    dmashm_rd.address = (physaddr_t)flash_buf;
    dmashm_rd.size = FLASH_BUF_SIZE;
    /* Crypto DMA will read from this buffer */
    dmashm_rd.mode = DMA_SHM_ACCESS_RD;

    dmashm_wr.target = id_dfucrypto;
    dmashm_wr.source = task_id;
    dmashm_wr.address = (physaddr_t)flash_buf;
    dmashm_wr.size = FLASH_BUF_SIZE;
    /* Crypto DMA will write into this buffer */
    dmashm_wr.mode = DMA_SHM_ACCESS_WR;

    printf("Declaring DMA_SHM for FLASH read flow\n");
    ret = sys_init(INIT_DMA_SHM, &dmashm_rd);
    printf("sys_init returns %s !\n", strerror(ret));

    printf("Declaring DMA_SHM for FLASH write flow\n");
    ret = sys_init(INIT_DMA_SHM, &dmashm_wr);
    printf("sys_init returns %s !\n", strerror(ret));

    get_flash_size();
    init_flash_map();

    /* now that firmware backend is declared, we have to
     * share the flash area with smart to allow it to check
     * the written firmware signature at the end */
    dmashm_flash.target = id_dfusmart;
    dmashm_flash.source = task_id;

    if (is_in_flip_mode()) {
        dmashm_flash.address = (physaddr_t)firmware_get_flop_base_addr();
    } else if (is_in_flop_mode()) {
        dmashm_flash.address = (physaddr_t)firmware_get_flip_base_addr();
    } else {
        printf("error: neither flip or flop mode detected!\n");
        goto err;
    }
    dmashm_flash.size = flash_size;
    /* Crypto DMA will write into this buffer */
    dmashm_flash.mode = DMA_SHM_ACCESS_RD;

    printf("Declaring DMA_SHM for FLASH memory device\n");
    ret = sys_init(INIT_DMA_SHM, &dmashm_flash);
    printf("sys_init returns %s !\n", strerror(ret));

#ifdef CONFIG_APP_DFUFLASH_USE_BKUP_SRAM
    /* Declare our Backup SRAM to get our flash key */ 
    if(bsram_flashkey_init()){
        goto err;
    }
#endif

    /*******************************************
     * End of init
     *******************************************/

    printf("set init as done\n");
    ret = sys_init(INIT_DONE);
    printf("sys_init returns %s !\n", strerror(ret));

    /* Set the LED off by default */
    led_off();
    /*******************************************
     * let's synchronize with other tasks
     *******************************************/
    logsize_t size = sizeof(struct sync_command);

    printf("sending end_of_init synchronization to dfucrypto\n");
    ipc_sync_cmd.magic = MAGIC_TASK_STATE_CMD;
    ipc_sync_cmd.state = SYNC_READY;

    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, size, (char*)&ipc_sync_cmd);
    } while (ret != SYS_E_DONE);

    /* Now wait for Acknowledge from Smart */
    id = id_dfucrypto;

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    if(ret != SYS_E_DONE){
        goto err;
    }
    if (   ipc_sync_cmd.magic == MAGIC_TASK_STATE_RESP
        && ipc_sync_cmd.state == SYNC_ACKNOWLEDGE) {
        printf("dfucrypto has acknowledge end_of_init, continuing\n");
    }

    /*******************************************
     * Starting end_of_cryp synchronization
     *******************************************/

    printf("waiting end_of_cryp syncrhonization from dfucrypto\n");

    id = id_dfucrypto;
    size = sizeof(struct sync_command);

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    if(ret != SYS_E_DONE){
        goto err;
    }

    if (   ipc_sync_cmd.magic == MAGIC_TASK_STATE_CMD
        && ipc_sync_cmd.state == SYNC_READY) {
        printf("dfucrypto module is ready\n");
    }

    /* init phase of drivers/libs */
    firmware_init();

    ipc_sync_cmd.magic = MAGIC_TASK_STATE_RESP;
    ipc_sync_cmd.state = SYNC_READY;
    size = sizeof(struct sync_command);
    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, size, (char*)&ipc_sync_cmd);
    } while (ret != SYS_E_DONE);
    // take some time to finish all sync ipc...
    sys_sleep(1000, SLEEP_MODE_INTERRUPTIBLE);

    /*******************************************
     * Sharing DMA SHM address and size with dfucrypto
     *******************************************/
    struct dmashm_info {
        uint32_t addr;
        uint16_t size;
    };

    struct dmashm_info dmashm_info;

    dmashm_info.addr = (uint32_t)flash_buf;
    dmashm_info.size = FLASH_BUF_SIZE;

    printf("informing dfucrypto about DMA SHM...\n");
    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, sizeof(struct dmashm_info), (char*)&dmashm_info);
    } while (ret != SYS_E_DONE);
    printf("Crypto informed.\n");

    /* All of initialization and syncrhonisation phase done,
     * going IDLE */
    set_task_state(DFUFLASH_STATE_IDLE);

    /*******************************************
     * Main read/write loop
     *   FLASH is waiting for READ/WRITE command
     *   from IPC interface
     *******************************************/

    printf("FLASH main loop starting\n");

    /*
     * Main waiting loop. The task main thread is awoken by any external
     * event such as ISR or IPC.
     */
    main_loop();

    while (1);
    return 0;
err:
    while (1);
    return 1;
}


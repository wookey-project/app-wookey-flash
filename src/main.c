/**
 * @file main.c
 *
 * \brief Main of dummy
 *
 */

#include "api/syscall.h"
#include "api/print.h"
#include "wookey_ipc.h"
#include "libfw.h"

uint32_t total_read = 0;

/*
 * We use the local -fno-stack-protector flag for main because
 * the stack protection has not been initialized yet.
 *
 * We use _main and not main to permit the usage of exactly *one* arg
 * without compiler complain. argc/argv is not a goot idea in term
 * of size and calculation in a microcontroler
 */
#define FLASH_DEBUG 0
#define FLASH_BUF_SIZE 4096

/* NOTE: alignment due to DMA */
__attribute__((aligned(4))) uint8_t flash_buf[FLASH_BUF_SIZE] = { 0 };

#define CRC 1

#ifdef CRC
volatile uint32_t crc_value = 0xffffffff;
volatile uint16_t block_num = 0;
#endif

int _main(uint32_t task_id)
{
    e_syscall_ret ret;
    char *wellcome_msg = "hello, I'm flash";
    struct sync_command ipc_sync_cmd;
    uint8_t id_dfucrypto;
    uint8_t id_dfusmart;
    uint8_t id;
    dma_shm_t dmashm_rd;
    dma_shm_t dmashm_wr;

    dma_shm_t dmashm_flash;

    printf("%s, my id is %x\n", wellcome_msg, task_id);

    ret = sys_init(INIT_GETTASKID, "dfucrypto", &id_dfucrypto);
    printf("dfucrypto is task %x !\n", id_dfucrypto);
    ret = sys_init(INIT_GETTASKID, "dfusmart", &id_dfusmart);
    printf("dfusmart is task %x !\n", id_dfusmart);

    /* Partition mode detection and flash base address and size computation */
    volatile physaddr_t addr_base = 0;
    volatile physaddr_t addr = 0;
    volatile uint32_t flash_size = 0;
    if (is_in_flip_mode()) {
        addr_base = 0x08120000;
        flash_size = firmware_get_flop_size();
    } else if (is_in_flop_mode()) {
        addr_base = 0x08020000;
        flash_size = firmware_get_flip_size();
    } else {
        printf("neither in flip or flop mode !!! leaving !\n");
        goto err;
    }


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

    firmware_early_init();

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


    /*******************************************
     * End of init
     *******************************************/

    printf("set init as done\n");
    ret = sys_init(INIT_DONE);
    printf("sys_init returns %s !\n", strerror(ret));

    /*******************************************
     * let's syncrhonize with other tasks
     *******************************************/
    logsize_t size = sizeof(struct sync_command);

    printf("sending end_of_init synchronization to dfucrypto\n");
    ipc_sync_cmd.magic = MAGIC_TASK_STATE_CMD;
    ipc_sync_cmd.state = SYNC_READY;

    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, size, (char*)&ipc_sync_cmd);
    } while (ret == SYS_E_BUSY);

    /* Now wait for Acknowledge from Smart */
    id = id_dfucrypto;

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
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
    } while (ret == SYS_E_BUSY);
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
    } while (ret == SYS_E_BUSY);
    printf("Crypto informed.\n");


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
      // FIXME:
      uint32_t buffer_count = 0;
      struct sync_command_data dataplane_command_wr;
      struct sync_command_data dataplane_command_ack = { 0 };
      t_ipc_command ipc_mainloop_cmd = { 0 };

      bool flash_is_mapped = false;
      while (1) {
          uint8_t id = id_dfucrypto;
          logsize_t size = sizeof(struct sync_command_data);

          ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_mainloop_cmd);

          if (ret != SYS_E_DONE) {
              continue;
          }

          switch (ipc_mainloop_cmd.magic) {


              case MAGIC_DATA_WR_DMA_REQ:
                  {
                      dataplane_command_wr = ipc_mainloop_cmd.sync_cmd_data;
		      block_num =  (uint16_t)dataplane_command_wr.data.u16[1];
#if FLASH_DEBUG
                      printf("!!!!!!!!!!! received DMA write command to FLASH: blocknum:%x size: %x\n",
                              (uint16_t)dataplane_command_wr.data.u16[1], (uint16_t)dataplane_command_wr.data.u16[0]);
/*
                      if (buffer_count < 2) {
                          printf("received buffer (4 start, 4 end)\n");
                          hexdump(flash_buf, 4);
                          hexdump(flash_buf+4092, 4);
                      }
*/
#endif
#if CRC
			crc_value = crc32(flash_buf, dataplane_command_wr.data.u16[0], crc_value);
#if FLASH_DEBUG
			printf("[CRC32] current crc is %x (chunk %x)\n", crc_value ^ 0xffffffff, block_num);
#endif
#endif
            /* let's write ! */
            uint32_t bufsize = (uint32_t)dataplane_command_wr.data.u16[0];
            if (!flash_is_mapped) {
                fw_storage_prepare_access();
                flash_is_mapped = true;
            }
	    addr = addr_base + (block_num * FLASH_BUF_SIZE);
#if FLASH_DEBUG
	    printf("Writing flash @%x, firmware block %d\n", addr, block_num);
#endif
            fw_storage_write_buffer(addr, (uint32_t*)flash_buf, bufsize);

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

              case MAGIC_DATA_RD_DMA_REQ:
                  {
                      uint16_t read_data = 0;
                      uint32_t prev_total = total_read;
                      dataplane_command_wr = ipc_mainloop_cmd.sync_cmd_data;
		      block_num = dataplane_command_wr.data.u16[1];
#if FLASH_DEBUG

                      printf("!!!!!!!!!!! received DMA read command to FLASH: blocknum:%x size: %x\n",
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
                      // read request.... let's read then...

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

            case MAGIC_DFU_DWNLOAD_FINISHED:
                {
#if CRYPTO_DEBUG
                    printf("receiving EOF from DFUUSB\n");
#endif

                    fw_storage_finalize_access();
                    dataplane_command_ack.magic = MAGIC_DFU_WRITE_FINISHED;
                    dataplane_command_ack.state = SYNC_DONE;

                    ret = sys_ipc(IPC_SEND_SYNC, id_dfucrypto, sizeof(struct sync_command), (const char*)&dataplane_command_ack);

                    if (ret != SYS_E_DONE) {
                        printf("Error ! unable to send back DFU_WRITE_FINISHED to crypto!\n");
                    }

                    break;
                }


              default:
                  {
                      printf("received invalid command from CRYPTO (magic: %d\n", ipc_mainloop_cmd.magic);
                      ipc_mainloop_cmd.magic = MAGIC_INVALID;
                      sys_ipc(IPC_SEND_SYNC, id_dfucrypto, sizeof(t_ipc_command), (const char*)&ipc_mainloop_cmd);
                      break;

                  }
          }
      }

    return 0;
err:
    return 1;
}


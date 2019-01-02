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

uint16_t blocknum = 0;
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
#define FLASH_SIZE 800000 // 800K flash size, test size
uint8_t flash_buf[FLASH_BUF_SIZE] = { 0 };

#define CRC 1

uint32_t crc_value = 0xffffffff;

int _main(uint32_t task_id)
{
    e_syscall_ret ret;
    char *wellcome_msg = "hello, I'm flash";
    struct sync_command ipc_sync_cmd;
    uint8_t id_dfucrypto;
    uint8_t id;
    dma_shm_t dmashm_rd;
    dma_shm_t dmashm_wr;

    printf("%s, my id is %x\n", wellcome_msg, task_id);

    ret = sys_init(INIT_GETTASKID, "dfucrypto", &id_dfucrypto);
    printf("dfucrypto is task %x !\n", id_dfucrypto);

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
    // flash_init();

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
     * Main waiting loopt. The task main thread is awoken by any external
     * event such as ISR or IPC.
     */
      // FIXME:
      uint32_t buffer_count = 0;
      struct sync_command_data dataplane_command_wr;
      struct sync_command_data dataplane_command_ack = { 0 };
      t_ipc_command ipc_mainloop_cmd = { 0 };

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
			printf("[CRC32] current crc is %x\n", crc_value ^ 0xffffffff);
#endif
#endif
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
#if FLASH_DEBUG

                      printf("!!!!!!!!!!! received DMA read command to FLASH: blocknum:%x size: %x\n",
                              blocknum, dataplane_command_wr.data.u16[0]);
#endif
                      total_read += dataplane_command_wr.data.u16[0];
                      if (total_read > FLASH_SIZE) {
                          /* reinit for next upload if needed */
                          total_read = 0;
                          blocknum = 0;
                          read_data = FLASH_SIZE - prev_total;
#if FLASH_DEBUG
                          printf("end of flash read, residual size is %x\n", read_data);
#endif
                      } else {
                          read_data = dataplane_command_wr.data.u16[0];
                          blocknum  += 1;
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
}


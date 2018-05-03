#include <rtthread.h>
#ifdef RT_USING_LWIP
#include <board.h>
#include <ctype.h>
//#include <dfs_posix.h>
#include <lwip/sockets.h>
 #include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
 


void http_server_serve(struct netconn *conn) 
{
  struct netbuf *inbuf;
  err_t recv_err;
  char* buf;
  u16_t buflen;

  while(1)
  {
	  /* Read the data from the port, blocking if nothing yet there. 
	   We assume the request (the part we care about) is in one netbuf */
	  recv_err = netconn_recv(conn, &inbuf);
	  //rt_kprintf("recv_err=%d\r\n",recv_err);
	  if (recv_err == ERR_OK)
	  {
	    if (netconn_err(conn) == ERR_OK) 
	    {
	      netbuf_data(inbuf, (void**)&buf, &buflen);
		  netbuf_delete(inbuf);
		  rt_kprintf("buflen=%d, buf = %s\r\n",buflen,buf);
	    }
	  }
  }
}


/**
  * @brief  http server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void http_server_netconn_thread(void *arg)
{ 
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, 80);
    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
  
      while(1) 
      {
        /* accept any icoming connection */
        accept_err = netconn_accept(conn, &newconn);
		rt_kprintf("accept_err = %d\r\n",accept_err);
        if(accept_err == ERR_OK)
        {
          /* serve connection */
          http_server_serve(newconn);

          /* delete connection */
          netconn_delete(newconn);
        }
      }
    }
  }
}

int http_server_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("HttpServer",
                           http_server_netconn_thread, RT_NULL,
                           800, RT_THREAD_PRIORITY_MAX-3, 5);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(http_server_init);
#endif

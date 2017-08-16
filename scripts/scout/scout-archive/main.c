/*
 * reactived.scout - a reactive deamon for the super scout.
 *
 * Note that this code has been rewritten from the various reactive
 * systems that went before it. The major change here has been to
 * simplify the code as (i) the onboard software has improved and (ii)
 * the old onboard code was overly complex. Time will tell.
 *
 * Michael Jenkin, January 1999.
 */

# include <stdio.h>
# include <stdlib.h>
# include <math.h>

# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <netdb.h>

# include "scout.h"

# define SOCKNAMELEN 80
# define SOCKETPORT 20000

static int initializeNetwork(void);


int main()
{
  int sock, msgsock;

  /* initialize network connection */
  sock = initializeNetwork();

  /* and react to connections */
  for(;;){
    fprintf(stderr,"reactived: Awaiting connection\n");
    if((msgsock = accept(sock, (struct sockaddr *)NULL, (int *)NULL)) < 0){
      perror("accept failed");
      exit(1);
    }
    fprintf(stderr,"reactived: Connection made (fd %d)\n", msgsock);
    react(msgsock);
    close(msgsock);
    fprintf(stderr,"reactived: Connection terminated\n");
  }
}

/* establish network link */
static int initializeNetwork()
{
  int length, sock, i;
  struct sockaddr_in server;
  char buf[SOCKNAMELEN], tbuf[80];

  /* establish our socket */
  if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
    perror("opening stream socket");
    exit(1);
  }

  /* take SOCKETPORT and re-use it */
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(SOCKETPORT);
  i = 1;
  (void) setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &i, sizeof(i));
  if(bind(sock, (struct sockaddr *)&server, sizeof(server))){
    perror("binding stream socket");
    exit(1);
  }
  length = sizeof(server);
  if(getsockname(sock, (struct sockaddr *)&server, &length)){
    perror("getting socket name");
    exit(1);
  }
  gethostname(buf, SOCKNAMELEN); buf[SOCKNAMELEN-1] = '\0';
  fprintf(stderr,"nreactived can be called on %s at %d\n",
	  buf, ntohs(server.sin_port));
  (void) sprintf(tbuf, "reactived on %s port %d",buf, ntohs(server.sin_port));
  if(listen(sock, 5) < 0){
    perror("listen failed");
    exit(1);
  }
  return(sock);
}

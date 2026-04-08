#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h> // for sched_yield()
#include <assert.h>
#include <sys/resource.h>
#include <math.h>
#include <dirent.h>
#include <sys/time.h>
#include <errno.h>
#include <stdint.h>
#include <sys/signal.h>
#include <termios.h>
#include <ctype.h>

int gs_quit = 0;
int gs_exit = 0;

float vref = 1.0;
float turn = 0.0;
float ref_right_wall = 1.0;
float left_turn_Start_threshold = 1.750;
float left_turn_Stop_threshold = 3.0;
float Kp_right_wall = -2.5;
float Kp_front_wall = -1.5;
float front_turn_velocity = 0.5;
float forward_velocity = 1;
float turn_command_saturation = 2;
float right_turn_threshold = 3.75;

int sem_count_send = 0; //for sem_getvalue
#define CMDNUM_FROM_FLOATS 11
#define RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME "/sem-LINUXCMDApp-recvfrom"
#define RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME "/sharedmem-LINUXCMDApp-recvfrom"

typedef union {
    char data_char[4*CMDNUM_FROM_FLOATS];
    float data_flts[CMDNUM_FROM_FLOATS];
} int_FromCMD_union;

struct shared_memory_recvfrom_LINUXCMDApp
{
  int_FromCMD_union new_FromCMD;
};

struct shared_memory_recvfrom_LINUXCMDApp *shared_mem_ptr_recvfrom_LINUXCMDApp;
sem_t *recvfrom_LINUXCMDApp_mutex_sem;
int recvfrom_LINUXCMDApp_fd_shm;


int mygetch(void)
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}


// Print system error and exit
void error (char *msg)
{
    perror (msg);
    exit (1);
}

/*  
* gs_killapp()
*   ends application safely
*
*/
void gs_killapp(int s)
{
	printf("\nTerminating\n");
	gs_quit = 1;
	gs_exit = 1;
	return;
}

/*
* main()
*   process command line input
*/
int main (int argc, char **argv)
{
	int i = 0;

	char buffer[200];  // used by fgets to read character string typed by user.
	char mychar;
	float tempfloat = 0;
	
	fflush(stdout);
	 
  	
    //create the semaphore for recv 
    if ((recvfrom_LINUXCMDApp_mutex_sem = sem_open(RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error("Error recv LINUXCMDApp sem_open");

    // create shared memory for recv
    if ((recvfrom_LINUXCMDApp_fd_shm = shm_open(RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error("Error shm_open LINUXCMDApp");

    //map the memory to virtual address
    if ((shared_mem_ptr_recvfrom_LINUXCMDApp = mmap(NULL, sizeof(struct shared_memory_recvfrom_LINUXCMDApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                                recvfrom_LINUXCMDApp_fd_shm, 0)) == MAP_FAILED)
        error("Error mmap LINUXCMDApp");

	printf("Setting signal handler...\n");
	signal(SIGKILL, gs_killapp);
	signal(SIGINT, gs_killapp);
	printf("...OK\n");
	printf(".\n");
	while (!gs_exit) {
		sched_yield();

		printf("\n\n");
		printf("Menu of Selections\n");
		printf("DO NOT PRESS CTRL-C when in this menu selection\n");
		printf("e - Exit Application\n");
		printf("s - enter Desired Velocity Setpoint and RightWall speed (ft/s)\n");
		printf("q - increment Left\n");
		printf("p - increment Right\n");
		printf("l - List All Parameters\n");
		// new stuff
		printf("r - set ref right wall\n");
		printf("t - left turn start threshold\n");
		printf("y - left turn stop threshold\n");
		printf("u - kp right wall\n");
		printf("i - kp front wall\n");
		printf("o - front turn vel\n");
		printf("j - forward vel\n");
		printf("k - turn command sat\n");
		printf("h - right_wall-far\n");

		mychar = (char) mygetch();
		
		switch (mychar) {
		case 'q':
			if (turn > 0.0) {
				turn = 0.0;
			} else {
				turn = turn - 0.2;
			}
			printf("turn =%.3f\n",turn);
			break;
		case 'p':                                
			if (turn < 0.0) {
				turn = 0.0;
			} else {
				turn = turn + 0.2;
			}
			printf("turn =%.3f\n",turn);
			break;
		case 'e':
			gs_exit = 1;
			break;
		case 's':
			printf("Enter Desired Velocity (vref) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					vref = tempfloat;
					printf("DVel = %.3f\n",vref);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'r':
			printf("Enter desired ref_right_wall value\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					ref_right_wall = tempfloat;
					printf("rrw = %.3f\n",vref);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 't':
			printf("Enter desired left turn start threshold\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					left_turn_Start_threshold = tempfloat;
					printf("start thresh = %.3f\n",vref);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;	
		case 'y':
			printf("Enter Desired left turn stop threshold\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					left_turn_Stop_threshold = tempfloat;
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'u':
			printf("Enter Desired Kp right wall\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					Kp_right_wall = tempfloat;
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'i':
			printf("Enter Desired Kp front wall\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					Kp_front_wall = tempfloat;
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'o':
			printf("Enter Desired front_turn_velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					front_turn_velocity = tempfloat;
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'j':
			printf("Enter Desired forward vel\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					forward_velocity = tempfloat;
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'k':
			printf("Enter Desired turn command sat\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					turn_command_saturation = tempfloat;
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'h':
			printf("Enter desired right turn threshold\n");
			fgets(buffer, 190, stdin);
			buffer[strlen(buffer) -1] = '\0';
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					right_turn_threshold = tempfloat;
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: right_wall-ref not changed\n");
			}
			
			break;

		case 'l':
			printf("\n");
			printf("turn = %.3f\n",turn);
			printf("vref = %.3f\n",vref);
			printf("ref_right_wall = %.3f\n",ref_right_wall);
			printf("left_turn_start = %.3f\n",left_turn_Start_threshold);
			printf("left_turn_end = %.3f\n",left_turn_Stop_threshold);
			printf("kp_right = %.3f\n",Kp_right_wall);
			printf("kp_front = %.3f\n",Kp_front_wall);
			printf("front_turn = %.3f\n",front_turn_velocity);
			printf("forward_vel = %.3f\n",forward_velocity);
			printf("turn_command_sat = %.3f\n",turn_command_saturation);
			printf("right_turn_threshold = %.3f\n", right_turn_threshold);
			break;	
		default:
			
			break;
		}
		
		shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[0] = vref;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[1] = turn;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[2] = ref_right_wall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[3] = left_turn_Start_threshold;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[4] = left_turn_Stop_threshold;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[5] = Kp_right_wall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[6] = Kp_front_wall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[7] = front_turn_velocity;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[8] = forward_velocity;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[9] = turn_command_saturation;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[10] = right_turn_threshold;

		if (sem_getvalue(recvfrom_LINUXCMDApp_mutex_sem,  &sem_count_send) == 0) {
			if (sem_post(recvfrom_LINUXCMDApp_mutex_sem) == -1){
				printf("Error LINUXCMDApp sem_post: recvfrom_mutex");
			}
		}
		
	}

}



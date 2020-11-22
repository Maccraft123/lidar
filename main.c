#include <math.h>
#include <GL/glut.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

uint8_t buf[36];
uint16_t dst[360];
uint16_t speed = 0;
uint16_t startAngle = 0;
uint16_t distance[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t quality[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int fd;

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		printf("error %d from tcgetattr\n", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tggetattr\n", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes\n", errno);
}

void idle()
{
	int frame = 0;
	
	read(fd, buf, sizeof buf);

	for (int x = 1; x < 36; x++)
	{
		if (buf[x] == 0x55 && buf[x+1] == 0xAA)
		{
			if (buf[x+2] == 0x3 && buf[x+3] == 0x8)
			{
				frame = 1;
				speed =  ((uint16_t) (buf[x+5] << 8) | buf[x+4]) / 64.0;
				startAngle = (uint16_t) (buf[x+7] << 8 | buf[x+6]) / 64 - 640;
				distance[0] = (uint16_t) (buf[x+9] << 8 | buf[x+8]);
				quality[0] = buf[x+10];
				if (quality[0] > 0)
					dst[startAngle] = distance[0]/10;

				distance[1] = (uint16_t) (buf[x+12] << 8 | buf[x+11]);
				quality[1] = buf[x+13];
				if (quality[0] > 0)
					dst[startAngle+1] = distance[1]/10;

				distance[2] = (uint16_t) (buf[x+15] << 8 | buf[x+14]);
				quality[2] = buf[x+16];
				if (quality[2] > 0)
					dst[startAngle+2] = distance[2]/10;

				distance[3] = (uint16_t) (buf[x+18] << 8 | buf[x+17]);
				quality[3] = buf[x+19];
				if (quality[3] > 0)
					dst[startAngle+3] = distance[3]/10;

				distance[4] = (uint16_t) (buf[x+21] << 8 | buf[x+20]);
				quality[4] = buf[x+22];
				if (quality[4] > 0)
					dst[startAngle+4] = distance[4]/10;

				distance[5] = (uint16_t) (buf[x+24] << 8 | buf[x+23]);
				quality[5] = buf[x+25];
				if (quality[5] > 0)
					dst[startAngle+5] = distance[5]/10;

				distance[6] = (uint16_t) (buf[x+27] << 8 | buf[x+26]);
				quality[6] = buf[x+28];
				if (quality[6] > 0)
					dst[startAngle+6] = distance[6]/10;

				distance[7] = (uint16_t) (buf[x+30] << 8 | buf[x+29]);
				quality[7] = buf[x+31];
				if (quality[7] > 0)
					dst[startAngle+7] = distance[7]/10;

			}
		}
	}

	if (frame == 0)
	{
		usleep(1);
	}
	


	//printf("speed: %d RPM angle: %d DEG distance: %d        \r", speed, startAngle, quality[0] >10 ? 0 : distance[0]);
	fflush(stdout);
	glutPostRedisplay();
}	

void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glLoadIdentity();
	glColor3f(1.0, 1.0, 1.0);
	glOrtho(-1, 1, -1, 1, -1, 1);
	for(uint16_t i = 0; i < 360; i++)
	{
//		glBegin(GL_QUADS);
//			glVertex2f(i+1, 1.0/dst[i]);
//			glVertex2f(i, 1.0/dst[i]);
//			glVertex2f(i, -1.0/dst[i]);
//			glVertex2f(i+1, -1.0/dst[i]);
//		glEnd();
		glBegin(GL_POINTS);
			glVertex2f(sin(i)*1/dst[i], cos(i)*1/dst[i]);
		glEnd();
	}
	glutSwapBuffers();
}

int main(int argc, char *argv[])
{
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
	{
		return 2;
	}

	set_interface_attribs(fd, B115200, 0);
	set_blocking(fd, 0);

//	uint16_t speed = 0;
//	uint16_t startAngle = 0;
//	uint16_t distance[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//	uint8_t quality[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutCreateWindow("a");
	
	glutDisplayFunc(render);
	glutIdleFunc(idle);
	glutMainLoop();

	return 0;
}

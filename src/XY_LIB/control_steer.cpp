#include "control_steer.h"

int steer_fd = -1;

void set_io_high(int io_fd)
{
	char ioval[1] = {0};
	int ret;
	
	ioval[0] = '1';
	ret = write(io_fd, ioval, 1);
	lseek(io_fd, 0, SEEK_SET);
	
}

void set_io_low(int io_fd)
{
	char ioval[1] = {0};
	int ret;
	
	ioval[0] = '0';
	ret = write(io_fd, ioval, 1);
	lseek(io_fd, 0, SEEK_SET);
}



int obtain_io(int *fd)
{
	/* request the controller of gpio158 */
	*fd = open(SYSFS_IO_EXPORT, O_WRONLY);
	if(*fd == -1)
	{
		printf("Open export failed.\n");
		goto error;
	}
	write(*fd, SYSFS_IO_EXPORT_VAL, sizeof(SYSFS_IO_EXPORT_VAL));
	close(*fd);

	/* set gpio158 direction */
	*fd = open(SYSFS_IO_DIR, O_WRONLY);
	if(*fd == -1)
	{
		printf("Open gpio158 diretion failed.\n");
		goto release;
	}
	write(*fd, SYSFS_IO_DIR_VAL, sizeof(SYSFS_IO_DIR_VAL));
	close(*fd);

	/* open gpio158 value file to write */
	*fd = open(SYSFS_IO_VAL, O_WRONLY);
	if(*fd == -1)
	{
		printf("Open gpio158 value failed.\n");
		goto release;
	}
	
	//set_io_high(*fd);

	return 0;
	
release:
	close(*fd);

error:
	return 1;
	
}

int release_io(int *fd)
{
	close(*fd);
	*fd = -1;
	
	return 0;
}

int obtain_control_io(int *fd)
{
	return obtain_io(fd);
}

int release_control_io(int *fd)
{
	return release_io(fd);
}

int XY_Load_Goods(void)
{
	if(0 == obtain_control_io(&steer_fd))
	{
		/* first rising edge */
		set_io_low(steer_fd);
		usleep(2000);
		set_io_high(steer_fd);

		/* wait some time */
		usleep(50000);
		
		/* second rising edge */
		set_io_low(steer_fd);
		usleep(2000);
		set_io_high(steer_fd);
		
		release_io(&steer_fd);
		return 0;
	}
	return 1;
}

int XY_Unload_Goods(void)
{
	if(0 == obtain_control_io(&steer_fd))
	{
		/* first rising edge */
		set_io_low(steer_fd);
		usleep(2000);
		set_io_high(steer_fd);

		/* wait some time */
		usleep(50000);
		
		/* second rising edge */
		set_io_low(steer_fd);
		usleep(2000);
		set_io_high(steer_fd);
		
		release_io(&steer_fd);
		return 0;
	}
	return 1;
}


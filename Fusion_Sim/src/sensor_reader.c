#include "../include/sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _GNU_SOURCE
#define _XOPEN_SOURCE 700

int read_oxts_csv(const char *filename, struct imu_sample *imus, int max_imus, struct gps_sample *gps, int max_gps)
{
    char line[1024];
    char *rest = line; 
    char *token;
    double values[26];
    int imu_count = 0;
    int gps_count = 0;
    int i = 0;       

    FILE *fp = fopen(filename, "r"); //opens the file
    if (!fp)
        return (perror("fopen"), -1);

    if (!fgets(line, sizeof(line), fp)) // Skips header line
        return ( fclose(fp), -1);

    while (fgets(line, sizeof(line), fp))
    {
        if (imu_count >= max_imus || gps_count >= max_gps)
            break;

        token = strtok(rest, ","); // Tokenize by commas
        while (token && i < 26)
        {
            values[i++] = atof(token);
            token = strtok(NULL, ",");
        }

        if (i < 24)     // must have at least 23 sensor fields (KITTI format) + rel_time
            return (fprintf(stderr, "Parse error: expected ≥24 fields, got %d\n", i), -1);

        // Map columns to variables the commented are unused
        double lat  = values[0];
        double lon  = values[1];
        double alt  = values[2];
        double vn   = values[6];
        double ve   = values[7];
        double vu   = values[10];
        double ax   = values[11];
        double ay   = values[12];
        double az   = values[13];
        double wx   = values[17];
        double wy   = values[18];
        double wz   = values[19];
        double rel_time = values[i-1];

        // Store GPS sample
        gps[gps_count].t   = rel_time;
        gps[gps_count].lat = lat;
        gps[gps_count].lon = lon;
        gps[gps_count].alt = alt;
        gps[gps_count].vn  = vn;
        gps[gps_count].ve  = ve;
        gps[gps_count].vu  = vu;
        gps_count++;

        // Store IMU sample
        imus[imu_count].t  = rel_time;
        imus[imu_count].ax = ax;
        imus[imu_count].ay = ay;
        imus[imu_count].az = az;
        imus[imu_count].wx = wx;
        imus[imu_count].wy = wy;
        imus[imu_count].wz = wz;
        imu_count++;
    }

    fclose(fp);

    if (imu_count < gps_count)
        return imu_count;
    else
        return gps_count;
}

        //double roll = values[3];
        //double pitch= values[4];
        //double yaw  = values[5];
        //double vf   = values[8];
        //double vl   = values[9];
        //double af   = values[14];
        //double al   = values[15];
        //double au   = values[16];
        //double wf   = values[20];
        //double wl   = values[21];
        //double wu   = values[22];
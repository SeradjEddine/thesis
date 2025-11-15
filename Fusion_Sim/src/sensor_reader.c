#include "../include/sensors.h"
#include <stdio.h>
#include <unistd.h> 
#include <stdlib.h>
#include <string.h>

#define _GNU_SOURCE
#define _XOPEN_SOURCE 700

int sensor_reader(const char *filename,
                  struct imu_sample *imus, int max_imus,
                  struct gps_sample *gps, int max_gps)
{
    FILE *fp = fopen(filename, "r");
    if (!fp)
        return (perror("fopen"), -1);

    char line[1024];
    int imu_count = 0;
    int gps_count = 0;

    // Skip header line
    if (!fgets(line, sizeof(line), fp))
        return (fclose(fp), -1);

    while (fgets(line, sizeof(line), fp))
    {
        if (imu_count >= max_imus || gps_count >= max_gps)
            break;

        // Tokenize by commas
        char *rest = line;
        char *token;
        double values[26]; // 23 original + 2 skipped + 1 rel_time
        int i = 0;

        while ((token = strtok_r(rest, ",", &rest)) && i < 26)
            values[i++] = atof(token);

        if (i < 24) // must have at least 23 sensor fields (KITTI format) + rel_time
        { 
            write( 2, "Parse error: expected ≥24 fields, got", 37);
            continue;
        }

        // Map columns to variables
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

        double rel_time = values[i-1]; // last column

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
        return (imu_count);
    else
        return (gps_count);
}


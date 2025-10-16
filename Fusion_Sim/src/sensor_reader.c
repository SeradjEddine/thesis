#include "../include/sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _GNU_SOURCE
#define _XOPEN_SOURCE 700

/**
 * Reads OXTS CSV (with relative time as the last column) and fills IMU and GPS arrays.
 * parsing using comma-splitting to handle all numeric columns.
 *
 * @param filename   Path to CSV file
 * @param imus       Output array for IMU samples
 * @param max_imus   Capacity of IMU array
 * @param gps        Output array for GPS samples
 * @param max_gps    Capacity of GPS array
 * @return           Number of samples successfully read (min of IMU/GPS count)
 */

int read_oxts_csv(const char *filename,
                  struct imu_sample *imus, int max_imus,
                  struct gps_sample *gps, int max_gps)
{
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("fopen");
        return -1;
    }

    char line[1024];
    int imu_count = 0;
    int gps_count = 0;

    // Skip header line
    if (!fgets(line, sizeof(line), fp)) {
        fclose(fp);
        return -1;
    }

    while (fgets(line, sizeof(line), fp)) {
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
            fprintf(stderr, "Parse error: expected ≥24 fields, got %d\n", i);
            continue;
        }

        // Map columns to variables
        double lat  = values[0];
        double lon  = values[1];
        double alt  = values[2];
        double roll = values[3];
        double pitch= values[4];
        double yaw  = values[5];
        double vn   = values[6];
        double ve   = values[7];
        double vf   = values[8];
        double vl   = values[9];
        double vu   = values[10];
        double ax   = values[11];
        double ay   = values[12];
        double az   = values[13];
        double af   = values[14];
        double al   = values[15];
        double au   = values[16];
        double wx   = values[17];
        double wy   = values[18];
        double wz   = values[19];
        double wf   = values[20];
        double wl   = values[21];
        double wu   = values[22];
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

    return imu_count < gps_count ? imu_count : gps_count;
}


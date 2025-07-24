#include <stdio.h>
#include <math.h>

void circle(double r, double angle_step_in_deg, double how_much_to_raise, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT %.12lf 0.0 %.12lf 0.0\n", r, original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / portion_of_circle * total_time + 0.5);
      double new_x = r * cos(angle);
      double new_y = r * sin(angle);
      double new_height = original_height + angle / portion_of_circle * how_much_to_raise;

      fprintf(f, "%d  POS  %.12lf %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_x,
		      new_y,
		      new_height,
		      angle); 

    }
    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

void circleCW(double r, double angle_step_in_deg, double how_much_to_raise, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT %.12lf 0.0 %.12lf 3.1415926536\n", r, original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / (portion_of_circle) * total_time + 0.5);
      double new_x = r * cos(angle);
      double new_y = -r * sin(angle);
      double new_height = original_height + angle / (portion_of_circle) * how_much_to_raise;

      fprintf(f, "%d  POS  %.12lf %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_x,
		      new_y,
		      new_height,
		      M_PI - angle); 

    }
    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

void circleV(double r, double angle_step_in_deg, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT 0.0 0.0 %.12lf 0.0\n", original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / (portion_of_circle) * total_time + 0.5);
      double new_height = r * cos(angle);
      double new_y = r * sin(angle);

      fprintf(f, "%d  POS  0.0 %.12lf %.12lf %.12lf\n", 
		      time_in_ms,
		      new_y,
		      original_height - r + new_height,
		      0.0); 

    }
    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

void circleVSIDE(double r, double angle_step_in_deg, double original_height, double total_time, double portion_of_circle, char *filename)
{
    FILE *f = fopen(filename, "w+");
    if (!f) 
    {
      perror("Cannot open output file");
      return;
    }

    fprintf(f, "#########################\n");
    fprintf(f, "PROCEDURE %s\n", filename);
    fprintf(f, "0 REFPOINT 0.0 0.0 %.12lf 0.0\n", original_height); 

    for (double angle = 0.0; angle < portion_of_circle; angle += angle_step_in_deg * M_PI / 180.0)
    {
      int time_in_ms = (int)(angle / (portion_of_circle) * total_time + 0.5);
      double new_height = r * cos(angle);
      double new_x = r * sin(angle);

      fprintf(f, "%d  POS  %.12lf 0.0 %.12lf %.12lf\n", 
		      time_in_ms,
		      new_x,
		      original_height - r + new_height,
		      0.0); 

    }
    fprintf(f, "ENDP\n");
    fprintf(f, "#########################\n");
    fclose(f);
}

int main()
{
    double r = 0.75;
    double angle_step_in_deg = 5.0;
    double how_much_to_raise = 1.0;
    double original_height = 1.5; 
    double total_time = 16000;
    double portion_of_circle = 2 * M_PI;

    circle(r, angle_step_in_deg, how_much_to_raise, original_height, total_time, portion_of_circle, "CIRCLE_UP_1M");
    circle(r, angle_step_in_deg, -1,                original_height, total_time, portion_of_circle, "CIRCLE_DOWN_1M");
    circle(r, angle_step_in_deg, 0,                 3.3,             total_time, portion_of_circle, "CIRCLE_LEVEL_CCW");
    circleCW(r, 5,               0,                 3.3,             total_time, portion_of_circle, "CIRCLE_LEVEL_CW");
    circle(r, 5,                 0,                 original_height, total_time / 4.0, M_PI / 2.0,  "ONEQ_LEVEL");
    circle(r, 5,                 0,                 original_height,   total_time / 2.0, M_PI,        "TWOQ_LEVEL");
    circle(r, 5,                 0,                 original_height, 3.0 * total_time / 4.0, 3 * M_PI / 2.0, "THREEQ_LEVEL");
    circle(r, angle_step_in_deg, 1.8,               original_height, total_time, portion_of_circle, "CIRCLE_UP_1M8");
    circle(r, angle_step_in_deg, -1.8,              3.3,             total_time, portion_of_circle, "CIRCLE_DOWN_1M8");
    circle(r, angle_step_in_deg, -1.8,              3.3,             3.0 * total_time / 4.0, 3 * M_PI / 2.0, "DOWN_34_1M8");
    circle(r, angle_step_in_deg, -1.8,              3.3,             total_time / 2.0, M_PI, "DOWN_12_1M8");
    circle(r, angle_step_in_deg, -1.8,              3.3,             total_time / 4.0, M_PI / 2.0, "DOWN_14_1M8");

    circleV(0.6, 6.0, 3.3, 9000, 2 * M_PI, "VERTICAL_FWD"); 
    circleVSIDE(0.6, 6.0, 3.3, 9000, 2 * M_PI, "VERTICAL_RIGHT"); 
}




#ifndef _UTILH_
#define _UTILH_

#include <string.h>

struct xyz{
	double x;
	double y;
	double z;
};

struct viz_thread_args{
	int *argc;
	char ***argv;
	bool terminate;
	bool draw_depth_filter;
	bool draw_pixel_match_color;
	bool verbose;
	struct xyz *jaco_tag_xyz;
	double *jaco_distances;
	int num_jaco_tags;
	struct xyz *object_xyz;
	double *object_distances;
	int num_objects;
};


// trim functions from http://stackoverflow.com/questions/122616/how-do-i-trim-leading-trailing-whitespace-in-a-standard-way
char *trimwhitespace(char *str){
  char *end;

  // Trim leading space
  while(isspace(*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace(*end)) end--;

  // Write new null terminator
  *(end+1) = 0;

  return str;
}

#endif


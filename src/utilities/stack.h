// stack.h - (c) Tyler Burdsall 2018
//
// Contains the function declarations for the stack data structure
#ifndef STACK_H
#define STACK_H
#include <stdlib.h>
#include <stdio.h>
#include "data_struct.h"

node*       create_node(point_xy*);
void        stack_push(node**, point_xy*);
point_xy*      stack_peek(node*);
point_xy*      stack_next_to_top(node*);
point_xy*      stack_pop(node**);
void        stack_print(node*);
void        stack_free(node**);
int        stack_count(node* head);
#endif

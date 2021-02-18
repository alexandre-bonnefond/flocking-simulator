// stack.c - (c) Tyler Burdsall 2018
//
// Contains the function definitions for the stack data structure.
#include "stack.h"

// Helper function to easily create a node given a
// point
node*
create_node(point_xy* p)
{
    node* n = (node*)malloc(sizeof(node));
    n->data = p;
    n->next = NULL;
    return n;
}

// Push a node onto a stack
void 
stack_push(node** head, point_xy* p)
{
    if (*head == NULL)
    {
        *head = create_node(p);
        return;
    }
    node* n = create_node(p);
    n->next = *head;
    *head = n;
    return;
}

// Returns a pointer to the point on the
// top of the stack
point_xy*
stack_peek(node* head)
{
    if(head == NULL)
    {
        return NULL;
    }
    return head->data;
}

// Returns a pointer to the 2nd point from the
// top, if it exists
point_xy*
stack_next_to_top(node* head)
{
    if(head == NULL)
    {
        return NULL;
    }
    if(head->next == NULL)
    {
        return NULL;
    }
    return head->next->data;
}


// Removes and returns the point from the top of the
// stack
point_xy*
stack_pop(node** head)
{
    if(*head == NULL)
    {
        return NULL;
    }
    node* t = *head;
    *head = (*head)->next;
    point_xy* p = t->data;
    free(t);
    t = NULL;
    return p;
}

// Recursively frees all of the memory allocated
// for a stack. If stack overflow issues were to occur,
// this could be switched to an iterative solution.
void
stack_free(node** head)
{
    if(*head == NULL)
    {
        return;
    }
    stack_free(&(*head)->next);
    free(*head);
    *head = NULL;
    return;
}

// Recursively prints the points in a stack in reverse-order. As mentioned
// before, if stack overflow issues begin to occur this can be switched
// to an iterative solution.
void
stack_print(node* head)
{
    if(!head)
    {
        return;
    }
    stack_print(head->next);
    printf("(%0.2f, %0.2f)  ", head->data->x, head->data->y);
}

int stack_count(node* head) {
    int i=0;
    while (head)
    {
        head = head->next;
        i++;
    }
    return i;
}

#include "FourD_Dict.h"
#include <stdlib.h>


FourD_Dict::FourD_Dict(){
  head = malloc(sizeof(struct Node));
  head->next = NULL;
  length = 0;
}

void FourD_Dict::putValue(int *key, float value){
  if (getValue(key) != NULL){
    updateValue(key, value);
  }
  else {
    Node *previous = NULL;
    Node *current = head;
    while (current != NULL){
      previous = current;
      current = current->next;
    }
    Node *temp = malloc(sizeof(struct Node));
    temp->next = NULL;
    temp->key = malloc(sizeof(int) * 4);
    temp->key[0] = key[0];
    temp->key[1] = key[1];
    temp->key[2] = key[2];
    temp->key[3] = key[3];
    temp->data = value;
    previous->next = temp;
    length++;
  }
}

float FourD_Dict::updateValue(int *key, float value){
  Node *current = head;
  while ((current->key)[0] != key[0] && (current->key)[1] != key[1] && (current->key)[2] != key[2] && (current->key)[3] != key[3]) current = current->next;
  float old_value = current->data;
  current->data = value;
  return old_value;
}

float FourD_Dict::getValue(int *key){
  Node *current = head;
  Node *previous = NULL;
  while (current != NULL){
    if ((current->key)[0]==key[0] && (current->key)[1]==key[1] && (current->key)[2]==key[2] && (current->key)[3]==key[3]){
      return current->data;
    }
    previous = current;
    current = current->next;
  }
  return NULL;
}

FourD_Dict::Node *FourD_Dict::getNode(int *key){
  Node *current = head;
  Node *previous = NULL;
  while (current != NULL){
    if ((current->key)[0]==key[0] && (current->key)[1]==key[1] && (current->key)[2]==key[2] && (current->key)[3]==key[3]){
      return current;
    }
    previous = current;
    current = current->next;
  }
  return NULL;
}

FourD_Dict::Node *FourD_Dict::getFirst2DKey (int *partialKey){
  Node *current = head;
  while (current != NULL){
    if ((current->key)[0] == partialKey[0] && (current->key)[1] == partialKey[1]){
      return current;
    }
    current = current->next;
  }
  return NULL;
}
    
FourD_Dict::Node *FourD_Dict::getLast2DKey(Node *first){
  Node *previous = first;
  int key1 = (first->key)[0];
  int key2 = (first->key)[1];
  first = first->next;
  while ((first->key)[0] == key1 && (first->key)[1] == key2){
    previous = first;
    first = first->next;
  }
  return previous;
}

float FourD_Dict::removeNode(int *key){
  Node *current = head;
  Node *previous = NULL;
  while (current != NULL){
    if ((current->key)[0]==key[0] && (current->key)[1]==key[1] && (current->key)[2]==key[2] && (current->key)[3]==key[3]){
      previous->next = current->next;
      float toReturn = current->data;
      free(current->key);
      free(current);
      return toReturn;
    }
    previous = current;
    current = current->next;
  }
  return NULL;
}
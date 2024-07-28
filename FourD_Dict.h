#ifndef FOURD_DICT
#define FOURD_DICT

class FourD_Dict {
public:
  int length;

  struct Node {
    Node *next;
    int *key;
    float data;
  };

  Node *head;

  FourD_Dict();
  float getValue(int *key);
  void putValue(int *key, float value);
  Node *getFirst2DKey(int *partialKey);
  Node *getLast2DKey(Node *first);  // REQUIRES that all keys (a,b,x,x) are next to each other
  Node *getNode(int *key); // returns the full node, instead of just the value
  float removeNode(int *key);

private:
  float updateValue(int *key, float value);
};

#endif
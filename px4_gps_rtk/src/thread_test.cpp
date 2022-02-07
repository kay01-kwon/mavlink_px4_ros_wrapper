#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>


void *p_function(void * data)
{
  pid_t pid; //process id
  pthread_t tid; // thread id

  pid = getpid(); //4
  tid = pthread_self();

  char* thread_name = (char *)data;
  int i = 0;

  while(i<3)
  {
    printf("thread name : %s, tid : %x, pid : %u\n", thread_name, (unsigned int)tid, (unsigned int)pid); //5
    i++;
    sleep(1);
  }
}


int main(void)
{
  pthread_t pthread[2];
  int thr_id;
  int status;
  char p1[] = "thread_1";
  char p2[] = "thread_2";
  char p3[] = "thread_3";

  sleep(1); //1

  thr_id = pthread_create(&pthread[0], NULL, p_function, (void*)p1); //2
  if(thr_id < 0)
  {
    perror("pthread0 create error");
    exit(EXIT_FAILURE);
  }

  thr_id = pthread_create(&pthread[1], NULL, p_function, (void *)p2); //2
  if(thr_id < 0)
  {
    perror("pthread1 create error");
    exit(EXIT_FAILURE);
  }

  p_function((void *)p3); //3

  pthread_join(pthread[0], (void **)&status); //6
  pthread_join(pthread[1], (void **)&status);

  printf("end??\n");
  return 0;
}
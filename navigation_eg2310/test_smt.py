# Python program to show time by perf_counter()
from time import perf_counter

# integer input from user, 2 input in single line
n, m = map(int, input().split())

# Start the stopwatch / counter
t1_start = perf_counter()

for i in range(n):
	t = int(input()) # user gave input n times
	if t % m == 0:
		print(t)

# Stop the stopwatch / counter
t1_stop = perf_counter()

print("Elapsed time:", t1_stop, t1_start)


print("Elapsed time during the whole program in seconds:", t1_stop-t1_start)

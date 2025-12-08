# Input File Format

The input file should have the following format:

1. **First line**: Starting address (integer)
2. **Following lines**: Instructions (one per line)
3. **"END" line**: Marks the end of instructions
4. **Memory initialization**: Address-value pairs (one pair per line)
5. **End marker**: `-1 -1` to finish memory initialization

## Example:

```
0
LOAD R1, 0(R0)
LOAD R2, 1(R0)
ADD R3, R1, R2
STORE R3, 2(R0)
END
0 10
1 20
-1 -1
```

This example:

- Starts at address 0
- Loads values from memory addresses 0 and 1 into R1 and R2
- Adds them and stores the result
- Initializes memory[0] = 10 and memory[1] = 20

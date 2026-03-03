#!/usr/bin/env python3

import sys
import numpy as np

def main() -> int:
    if 2 > len(sys.argv):
        return 1

    path = sys.argv[1]
    
    repeating = 1
    if 3 <= len(sys.argv):
        repeating = int(sys.argv[2])

    with open(f'{path}/poses.tsv', 'w') as f:
        try:
            i = 1
            while True:
                data = np.load(f'{path}/pose_{i}.npy')
                for j in range(0, repeating):
                    if 1 < (i + j):
                        f.write('\n')
                    np.savetxt(f, np.transpose(data), delimiter='', newline='\t', fmt="%.8f")
                i += 1
        except OSError as error:
            pass 

    return 0

if __name__ == '__main__':
    sys.exit(main())

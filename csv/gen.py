import random
import sys

def main() -> None:
    with open(sys.argv[1], "w", encoding="utf-8") as file:
        file.write(f"x,y\n")
        for _ in range(0, 20):
            file.write(f"{random.uniform(-5.0, 5.0)},{random.uniform(-5.0, 5.0)}\n")

    
if __name__ == "__main__":
    main()
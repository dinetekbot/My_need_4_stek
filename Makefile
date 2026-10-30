##
## lidar-racer -- autonomous car AI driven by a 32-ray lidar
##
## Sources are listed explicitly rather than globbed. A wildcard picks up
## whatever happens to be lying in the directory, which is how a stray file
## from an unrelated project ends up breaking a build that used to work.
##

NAME        := ai
TEST_NAME   := unit_tests

CC          := gcc
CFLAGS      := -std=c99 -Wall -Wextra -Wpedantic -O2 -Iinclude -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE
LDLIBS      := -lm

## -MMD -MP makes the compiler emit a .d file recording which headers each
## object actually included, and those files are pulled back in below.
##
## Without this, editing a header rebuilds nothing. Changing a field in a
## struct then links objects compiled against two different memory layouts,
## and the program misbehaves in ways that look like logic bugs and survive
## a rebuild. It cost an afternoon once; it costs two lines to prevent.
CFLAGS      += -MMD -MP

SRC         := src/main.c \
               src/car.c \
               src/protocol/link.c \
               src/protocol/response.c \
               src/protocol/commands.c \
               src/util/util.c

## Everything except main.c, so the tests can link the real implementation
## instead of a copy that drifts away from it.
LIB_SRC     := $(filter-out src/main.c,$(SRC))

TEST_SRC    := tests/main.c \
               tests/test_response.c \
               tests/test_car.c \
               tests/test_util.c

OBJ         := $(SRC:.c=.o)
LIB_OBJ     := $(LIB_SRC:.c=.o)
TEST_OBJ    := $(TEST_SRC:.c=.o)
DEPS        := $(OBJ:.o=.d) $(TEST_OBJ:.o=.d)

.PHONY: all clean fclean re test debug

all: $(NAME)

$(NAME): $(OBJ)
	$(CC) $(OBJ) -o $(NAME) $(LDLIBS)

## The tests exercise the pure functions: parsing, geometry, control laws.
## They need no simulator, no network and no fixtures, so they run in CI and
## in the container.
test: $(TEST_NAME)
	./$(TEST_NAME)

$(TEST_NAME): $(LIB_OBJ) $(TEST_OBJ)
	$(CC) $(LIB_OBJ) $(TEST_OBJ) -o $(TEST_NAME) $(LDLIBS)

debug: CFLAGS += -g3 -fsanitize=address,undefined -O0
debug: LDLIBS += -fsanitize=address,undefined
debug: fclean $(NAME)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJ) $(TEST_OBJ) $(DEPS)

fclean: clean
	$(RM) $(NAME) $(TEST_NAME)

re: fclean all

## Pull in the generated header dependencies. The leading '-' keeps the very
## first build, when no .d files exist yet, from failing.
-include $(DEPS)

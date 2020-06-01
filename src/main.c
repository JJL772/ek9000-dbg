/*
 *
 * Simple EK9000 simulator
 *
 */ 
#include <modbus/modbus.h>
#include <modbus/modbus-tcp.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdatomic.h>
#include <memory.h>
#include <time.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>

/* UNIX includes */
#include <unistd.h>
#include <pthread.h>
#include <getopt.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>

/* Readline includes */
#include <readline/readline.h>
#include <readline/history.h>

#include "term.h"

/* For some reason strdup doesnt exist everywhere */
char* strdup_(const char* str)
{
	size_t l = strlen(str);
	char* c = malloc(l) + 1;
	memcpy(c, str, l);
	c[l] = '\0';
	return c;
}

int strisspace(const char* str)
{
	for(char c = str[0]; c; c = *str++)
		if(!isspace(c)) return 0;
	return 1;
}

#define PRINT_AND_EXIT(str, code) { printf("%s\n", str); exit(code); }
#define PRINT_HELP_AND_EXIT(str, code) { printf("%s\n", str); ShowHelp(); exit(code); };
#define PRINT_AND_EXIT_VA(str, code, ...) { printf(str, __VA_ARGS__); exit(code); }
#define EXEC_AND_EXIT(fn, code) { fn; exit(code); }
#define CHARS_TO_INT16(c1, c2) ((c1) | ((c2) << 7))

#define BUS_COUPLER_ID 9000
#define BUS_COUPLER_ID_REG 0x1000

/* Default port is 1502 because we need root access to use 502 */
#define DEFAULT_PORT 1502

#define PDO_AO_SIZE_REG 0x1010
#define PDO_AI_SIZE_REG 0x1011
#define PDO_DO_SIZE_REG 0x1012
#define PDO_DI_SIZE_REG 0x1013
#define WATCHDOG_CURRENT_TIME_REG 0x1020
#define FALLBACKS_TRIGGERED_REG 0x1021
#define TCP_CONN_NUM_REG 0x1022
#define HARDWARE_VERSION_REG 0x1030
#define SOFTWARE_VERSION_MAIN_REG 0x1031
#define SOFTWARE_VERSION_SUBMAIN_REG 0x1032
#define SOFTWARE_VERSION_BETA_REG 0x1033
#define SERIAL_NUM_REG 0x1034

#define SOFTWARE_VERSION_MAIN 69
#define SOFTWARE_VERSION_SUBMAIN 420
#define SOFTWARE_VERSION_BETA 21
#define HARDWARE_VERSION 69
#define SERIAL_NUM 420

#define MFG_DATE_DAY_REG 0x1035
#define MFG_DATE_DAY 9

#define MFG_DATE_MONTH_REG 0x1136
#define MFG_DATE_MONTH 8

#define MFG_DATE_YEAR_REG 0x1037
#define MFG_DATE_YEAR 2019

#define WATCHDOG_TIME_REG 0x1120
#define WATCHDOG_TIME_DEFAULT 1000

#define WATCHDOG_TYPE_CONF_REG 0x1122 
#define WATCHDOG_TYPE_DISABLED 0
#define WATCHDOG_TYPE_WRITE  2
#define WATCHDOG_TYPE_TELEGRAM 1
#define WATCHDOG_TYPE_DEFAULT WATCHDOG_TYPE_TELEGRAM

#define FALLBACK_CONF_REG 0x1123
#define FALLBACK_SET_ZERO 0
#define FALLBACK_FREEZE 1
#define FALLBACK_STOP_EBUS 2
#define FALLBACK_DEFAULT FALLBACK_SET_ZERO 

#define WATCHDOG_RESET_REG 0x1121

#define WRITELOCK_CONF_REG 0x1124
#define WRITELOCK_ON 1
#define WRITELOCK_OFF 0

#define EBUS_CONF_REG 0x1140
#define EBUS_INIT 0
#define EBUS_OP 1

/* Note: these are in the form 0xXX2 or 0xXX5, etc. because we want to catch non-compliant code */
#define STATUS_ERROR 0x102
#define STATUS_BUSY 0x201
#define STATUS_DONE 0x405
#define STATUS_EXECUTE 1

#define EK9000_MAX_CONNECTIONS 2

#define EK9000_HOLDING_REG_START 0x0
#define EK9000_HOLDING_REG_NUM (0x14FF - EK9000_HOLDING_REG_START)
#define EK9000_INPUT_REG_START 0x0
#define EK9000_INPUT_REG_NUM (0x8000 - EK9000_INPUT_REG_START)
#define EK9000_COIL_START 0x0
#define EK9000_COIL_NUM 0x800
#define EK9000_INPUT_BIT_START 0x0
#define EK9000_INPUT_BIT_NUM 0x800

#define COLOR_GREEN 1
#define COLOR_NONE 0
#define COLOR_YELLOW 2
#define COLOR_RED 3

/* Command descriptor */
typedef struct
{
	const char* aliases;
	int naliases;
	void(*pfnCommand)(int,char**);
	const char* helptext;
} command_t;

#define COMMAND(name, pfn, help) { name, 1, pfn, help }

/* Command prototypes */
void command_exit(int,char**);
void command_reset_wdt(int,char**);
void command_reset_defaults(int,char**);
void command_help(int,char**);
void command_set_mfg_date(int,char**);
void command_set_sn(int, char**);
void command_set_hardware_ver(int, char**);
void command_set_soft_ver(int, char**);
void command_set_wdt_time(int, char**);
void command_set_fallback_mode(int, char**);
void command_print_settings(int, char**);
void command_print_info(int,char**);

command_t g_commands[] =
{
	COMMAND("exit", command_exit, "Exits the simulator"),
	COMMAND("reset_wdt", command_reset_wdt, "Resets the watchdog timer"),
	COMMAND("reset_defaults", command_reset_defaults, "Resets the device to device defaults"),
	COMMAND("help", command_help, "Displays help info"),
	COMMAND("set_wdt_time", command_set_wdt_time, "Sets the time until the watchdog triggers"),
	COMMAND("set_fallback_mode", command_set_fallback_mode, "Sets the fallback mode of the watchdog"),
	COMMAND("print_settings", command_print_settings, "Prints out the current settings of the device."),
	COMMAND("print_info", command_print_info, "Prints all R/O Info of the device"),
};
int g_ncommands = sizeof(g_commands) / sizeof(command_t);

/* Globals */
modbus_t* modbus_context;
FILE* log_handle;
int terminal_count = 0;
char* ip, *logfile;
int port = DEFAULT_PORT;
pthread_t listen_thread;
pthread_attr_t thread_attr;
int verbose;
modbus_mapping_t* modbus_map;
term_t** terms;
term_context_t** contexts;
pthread_t timer_thread;
pthread_mutex_t modbus_mutex; 
int dirty = 0; /* Used to mark modbus register space as dirty */
int wdt_triggered = 0, frozen = 0, nterms = 0, fallback_status;


/* Function prototypes */
int 	srv_start_server(const char* ip, int port);
int 	srv_stop_server();
void 	srv_server_loop();
void 	srv_open_shell();
void 	setup_terminals();
void* 	srv_listen(void*);
void 	cmd_show_help();
void 	init_regs();
void 	Log_Info(const char* fmt, ...);
void 	Log_Warn(const char* fmt, ...);
void 	Log_Err(const char* fmt, ...);
void 	Log_Fatal(const char* fmt, ...);
void	PrettyPrint(int color, const char* fmt, ...);
bool 	VerifyIP(const char* ip);
void 	InitMapping();
void* 	ek9000_update(void*);
void	ek9000_trigger_fallback();
void    ek9000_reset();
void    ek9000_stop_ebus();
int     ismemzero(void*,size_t);
int     create_noaccess_region(size_t start, size_t size, const char* name);
int     check_noaccess_regions();
int     check_regions();

typedef struct mem_region_s
{
	size_t start;
	size_t size;
	unsigned int flags;
	void* prevdat;
	const char* name;
#define REGION_FLAG_RO 1
#define REGION_FLAG_RW 2
#define REGION_FLAG_NOACCESS 3
} mem_region_t;

static mem_region_t g_regions[32];


int ismemzero(void* m, size_t sz)
{
	for(; sz > 0; --sz)
		if(((char*)m)[sz]) return 0;
	return 1;
}

int create_noaccess_region(size_t start, size_t end, const char* name)
{
	int i;
	for(i = 0; g_regions[i].start; i++);
	
	if(i >= 32) return -1;

	mem_region_t* reg = &g_regions[i];
	reg->start = start;
	reg->size = end-start;
	reg->flags = REGION_FLAG_NOACCESS;
	reg->prevdat = malloc(reg->size);
	reg->name = name;
	memcpy(reg->prevdat, (void*)start, end-start);
	return 0;
}

int check_regions()
{
	size_t addr = 0;
	uint16_t prev = 0, new = 0;
	for(int i = 0; i < 32; i++)
	{
		if(g_regions[i].size == 0) continue;
		mem_region_t* reg = &g_regions[i];
		/* Check the memory to verify that it's identical */
		if(memcmp((void*)reg->start, reg->prevdat, reg->size))
		{
			/* If it's not, let's go ahead and find the exact offending address */
			for(uint16_t* offset = (uint16_t*)reg->start;;offset++)
			{
				if(*offset != ((uint16_t*)reg->prevdat)[(size_t)offset-(reg->start/2)])
				{
					addr = (size_t)offset;
					prev = ((uint16_t*)reg->prevdat)[(size_t)offset-reg->start];
					new = *offset;
					break;
				}
			}
			Log_Err("MEMORY VALIDATION FAILED!\n");
			printf("-------------------------------\n");
			printf("Offending memory region: %s\n", reg->name);
			printf("Region base:       0x%lX\n", reg->start);
			printf("Region size:       0x%lX\n", reg->size);
			printf("Offending address: 0x%lX", addr);
			printf("Previous value:    0x%X\n", prev);
			printf("New value:         0x%X\n", new);
			printf("-------------------------------\n");
		}
	}
	return 0;
}


/* Entry point */
int main(int argc, char** argv)
{
	int ret;

	/* Get all options */
	for(int i = 0; i < argc; i++)
	{
		size_t arglen = strlen(argv[i]);
		if(strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
		{
			cmd_show_help();
			exit(0);
		}
		/* -l will set the log file */
		else if(strcmp(argv[i], "-l") == 0)
		{
			if(i < argc-1)
			{
				logfile = strdup_(argv[i+1]);
				log_handle = fopen(logfile, "w+");
				if(!log_handle)
					PRINT_AND_EXIT("Failed to open log file", 1);
				Log_Info("Opened log file\n");
				continue;
			}
			else
				PRINT_AND_EXIT("Please provide a log file name", 1);
		}
		/* More verbose method of setting log file */
		else if(strncmp(argv[i], "--log-file=", 11) == 0)
		{
			logfile = strdup_(argv[i] + 11);
			if(!logfile) 
				PRINT_AND_EXIT("Unable to open log file", 1);
			log_handle = fopen(logfile, "w+");
			if(!log_handle)
				PRINT_AND_EXIT("Failed to open log file", 1);
			Log_Info("Opened log file\n");
		}
		/* Specifies terminal list */
		else if(strncmp(argv[i], "--terminals=", 12) == 0)
		{
			char* tlist = strdup_(argv[i] + 12);
			if(!tlist)
				PRINT_AND_EXIT("Terminal list invalid.", 1);
			/* Alloc and zero our globals */
			terms = malloc(sizeof(term_t) * 256);
			memset(terms, 0, sizeof(term_t) * 256);
			/* Loop through comma delimited list of terminals */
			int i = 0;
			for(char* subst = strtok(tlist, ","); subst; subst = strtok(NULL, ","), i++)
			{
				int x = 0, found = 0;
				term_t* theone = 0;
				for(term_t* term = g_terms[0]; term; term = g_terms[++x])
				{
					if(strncmp(term->name, subst, strlen(term->name)) == 0){ found = 1; theone = term; break; }
				}
				if(!found)
					PRINT_AND_EXIT_VA("Terminal %s is not supported in this simulator.\n", 1, subst);
				terms[i] = theone;
				++nterms;
				assert(theone);
			}
		}
		/* More verbose method of specifying port */
		else if(strncmp(argv[i], "--port=", 7) == 0)
		{
			port = atoi(argv[i] + 7);
			if(errno != 0)
				PRINT_AND_EXIT("Invalid port number", 1);
		}
		/* Enables verbosity */
		else if(strncmp(argv[i], "-v", 2) == 0)
		{
			verbose = 1;
		}
		/* Lists out all the supported terminals */
		else if(strcmp(argv[i], "--terminal-list") == 0)
		{
			printf("Supported terminals:\n");
			int i = 0;
			for(term_t* term = g_terms[i]; term; term = g_terms[++i])
			{
				printf("\t%s\n", term->name);
			}
			exit(0);
		}
	}
	/* If no log handle has been opened, we will just use stdout */
	if(!log_handle)
		log_handle = stdout;
	
	/* Setup modbus mappings here */
	modbus_map = modbus_mapping_new_start_address(EK9000_COIL_START, EK9000_COIL_NUM,
		EK9000_INPUT_BIT_START, EK9000_INPUT_BIT_NUM, 
		EK9000_HOLDING_REG_START-1, EK9000_HOLDING_REG_NUM,
		EK9000_INPUT_REG_START, EK9000_INPUT_REG_NUM);

	/* Create our protected memory regions */
	create_noaccess_region((size_t)&modbus_map->tab_input_registers[0x800], 
		(size_t)&modbus_map->tab_input_registers[0x1000-1], "Protected Input Register Space\n");

	if(!modbus_map)
		PRINT_AND_EXIT("Failed to create modbus mapping.\n", 1);

	/* map the registers to the actual allocated memory space */
	init_regs();

	setup_terminals();

	/* Startup the server */
	ret = srv_start_server(ip, port);

	if(ret != 0)
		PRINT_AND_EXIT("Failed to start server. Exiting now.\n", 1);

	/* Open up the shell */
	srv_open_shell();

	/* Stop the server */
	srv_stop_server();

	/* Free everything */
	free(terms);
	free(contexts);
	modbus_mapping_free(modbus_map);
	modbus_free(modbus_context);

	return 0;
}

void setup_terminals()
{
	assert(modbus_map);
	/* Allocate space for all terminal contexts */
	contexts = malloc(sizeof(term_context_t*) * 256);
	memset(contexts, 0, sizeof(term_context_t*) * 256);
	
	/* Warn if no terminals on the rail */
	if(!terms)
	{
		Log_Warn("No terminals specified, so none will be mapped.\n");
		return;
	}
	term_t* table = terms[0];
	int pdo_ai_pos = 0, pdo_ao_pos = 0, pdo_di_pos = 0, pdo_do_pos = 0, ret = 0;

	/* Pass 0 (Analog mapping) */
	for(int i = 0; i < 256 && terms[i]; table = terms[++i])
	{
		assert(table);
		assert(table->pfnInit);
		/* Allocate context for this terminal */
		contexts[i] = malloc(sizeof(term_context_t));
		memset(contexts[i], 0, sizeof(term_context_t));
		/* Analog terms mapped first */
		if(table->type == TYPE_ANALOG)
		{
			contexts[i]->pdo_ai_buf = ((char*)modbus_map->tab_input_registers) + pdo_ai_pos;
			contexts[i]->pdo_ao_buf = ((char*)modbus_map->tab_registers) + pdo_ao_pos;
			ret = table->pfnInit(contexts[i],
				contexts[i]->pdo_ai_buf, contexts[i]->pdo_ao_buf, 0, 0);
			pdo_ai_pos += table->pdo_size_ai;
			pdo_ao_pos += table->pdo_size_ao;
			assert(ret == 0);
		}
	}
	table = terms[0];
	/* Pass 1 (Digital mapping) */
	for(int i = 0; i < 256 && terms[i]; table = terms[++i])
	{
		if(table->type == TYPE_DIGITAL)
		{
			contexts[i]->pdo_di_buf = ((char*)modbus_map->tab_input_registers) + pdo_di_pos;
			contexts[i]->pdo_do_buf = ((char*)modbus_map->tab_bits) + pdo_do_pos;
			ret = table->pfnInit(contexts[i],
				0, 0, contexts[i]->pdo_di_buf, contexts[i]->pdo_do_buf);
			pdo_di_pos += table->pdo_size_di;
			pdo_do_pos += table->pdo_size_do;
			assert(ret == 0);
		}
	}
	*reg_pdo_size_ai = pdo_ai_pos * 8;
	*reg_pdo_size_ao = pdo_ao_pos * 8;
	*reg_pdo_size_di = pdo_di_pos;
	*reg_pdo_size_do = pdo_do_pos;
}

int srv_start_server(const char* _ip, int _port)
{
	int ret;
	Log_Info("Starting server on %s:%u\n", _ip, _port);

	modbus_context = modbus_new_tcp(NULL, _port);

	if(!modbus_context)
	{
		Log_Err("Failed to create modbus context on %s:%u\n", _port, _ip);
		return 1;
	}

	Log_Info("Creating new listen thread.\n");

	pthread_attr_init(&thread_attr);
	ret = pthread_create(&listen_thread, NULL, srv_listen, NULL);

	if(ret)
	{
		Log_Err("Failed to create listen thread.\n");
		return 1;
	}

	return 0;
}

/* Listens for new connections and data on the socket */
void* srv_listen(void* v)
{
	int sock, rc, ret, maxfd;
	fd_set refset;
	char query[MODBUS_TCP_MAX_ADU_LENGTH];

	/* Start listen */
	sock = modbus_tcp_listen(modbus_context, 1);
	maxfd = sock;
	Log_Info("[Listen thread] Listening for connections. Srv socket=%u\n", sock);

	if(sock == -1)
	{
		Log_Err("Unable to listen for connections. Errno=%u\n", errno);
		raise(SIGABRT);
		return NULL;
	}

	FD_ZERO(&refset);
	FD_SET(sock, &refset);
	while(1)
	{
		//if(select(maxfd + 1, &refset, NULL, NULL, NULL) == -1)
		{
		//	Log_Err("[Listen thread] Error while accepting connection. Errno=%u [Select failed]\n", errno);
		//	continue;
		}
		for(int i = 0; i <= maxfd; i++)
		{
			if(!FD_ISSET(i, &refset)) continue;
			/* Asking for a connection, and it will signal the server listener socket */
			if(i == sock)
			{
				if(((*reg_connections) > 0 && (*reg_writelock)) || (!(*reg_writelock) && (*reg_connections) > 1))
				{
					Log_Warn("Max number of TCP connections reached, and another client is attempting to connect!\n");
					FD_CLR(i, &refset);
					continue;
				}
				int newconnfd = 0;
				struct sockaddr_in addr;
				socklen_t addrlen;

				newconnfd = accept(i, (struct sockaddr*)&addr, &addrlen);

				if(newconnfd == -1)
				{
					Log_Err("Unable to accept the connection.\n");
					continue;
				}
				++(*reg_connections);
				if(newconnfd > maxfd) maxfd = newconnfd;
				FD_SET(newconnfd, &refset);
				FD_CLR(i, &refset);
				Log_Info("Accepted new connection from %s:%u\n", inet_ntoa(addr.sin_addr), addr.sin_port);
			}
			/* Otherwise, we've got data on the other connections */
			else
			{
				modbus_set_socket(modbus_context, i);
				rc = modbus_receive(modbus_context, query);
				/* rc will be -1 if a client disconnects */
				if(rc == -1)
				{
					close(i);
					Log_Info("Connection closed on socket %u\n", i);
					FD_CLR(i, &refset);
					--(*reg_connections);
					if(maxfd == i) maxfd--;
					continue;
				}

				modbus_header_t header = *(modbus_header_t*)query;

				/* Correct the byte-order for 16-bit values */
				header.tid = ntohs(header.tid);
				header.length = ntohs(header.tid);
				header.start_addr = ntohs(header.start_addr);
				header.addr_cnt = ntohs(header.addr_cnt);

				/* Check if we should trigger the watchdog */
				if(*reg_wdt_type == WATCHDOG_TYPE_TELEGRAM)
				{
					*reg_wdt_curr_time = *reg_wdt_time;
					fallback_status = 0;
					wdt_triggered = 1;
				}
				else if(*reg_wdt_type = WATCHDOG_TYPE_WRITE && (header.function == 5 ||
					header.function == 15 || header.function == 6 || header.function == 16))
				{
					*reg_wdt_curr_time = *reg_wdt_time;
					fallback_status = 0;
					wdt_triggered = 1;
				}

#ifdef _DEBUG 
				printf("Modbus request received. Function=%u,slave=%u,startaddr=0x%X,num=0x%X\n",
					header.function, header.unitid, header.start_addr, header.addr_cnt);
#endif

				modbus_reply(modbus_context, query, rc, modbus_map);
				dirty = 1;
			}
		}
		/* Now that we've read all of our modbus requests, update the device itself */
		ek9000_update(NULL);
		/* Sleep for 1ms */
		usleep(1000);
	}
}

void srv_server_loop()
{

}

/* Opens an interactive shell for the programmer */
void srv_open_shell()
{
	/* Sleep for 1 second so we can avoid the server's output spew */
	sleep(1);
	//char input[4096];
	char* input;
	char* args[256];
	int ran = 0;

	/* Bind keys to readline actions */
	rl_bind_key('\t', rl_complete);
	PrettyPrint(COLOR_GREEN, "---- EK9000 Simulator Shell V1.0 ----\n");
	while(1)
	{
		input = readline("ek9000> ");
		/* If it's all spaces, just fail */
		if(strisspace(input)) goto done;
		for(int i = 0; i < g_ncommands; i++)
		{
			size_t cmdlen = strlen(g_commands[i].aliases);
			if(strncmp(input, g_commands[i].aliases, cmdlen) == 0)
			{
				/* Tokenize args */
				int n = 0;
				for(char* c = strtok(input, " "); c; c = strtok(NULL, " "), n++)
					args[n] = c;
				assert(g_commands[i].pfnCommand);
				/* Run command */
				g_commands[i].pfnCommand(n, args);
				goto done;
			}
		}
		add_history(input);
	nocmd:
		if(!ran) { printf("Command not found.\n"); ran = 0; };
	done:
		free(input);
		continue;
	}
}

int srv_stop_server()
{
	Log_Info("Killing listen thread\n");
	pthread_cancel(listen_thread);
	Log_Info("Stopping server\n");
	return 0;
}

void cmd_show_help()
{
	printf("USAGE: eksim [options] ip:port\n");
	printf("\t--log-file=<file>         - Log file to use\n");
	printf("\t   -l <file>\n");
	printf("\t--terminals=<term1,term2,...> - Terminals to use\n.");
}

void Log_Info(const char* fmt, ...)
{	
	va_list list;
	time_t _time = time(NULL);
	struct tm* _tm = localtime(&_time);
	char buf[512];
	sprintf(buf, "%u:%02u:%02u [INFO] %s", _tm->tm_hour, _tm->tm_min, _tm->tm_sec, fmt);

	va_start(list, fmt);
	vfprintf(log_handle, buf, list);
	va_end(list);
}

void Log_Warn(const char* fmt, ...)
{
	va_list list;
	time_t _time = time(NULL);
	struct tm* _tm = localtime(&_time);
	char buf[512];
	sprintf(buf, "\e[93m%u:%02u:%02u [WARN] %s\e[39m", _tm->tm_hour, _tm->tm_min, _tm->tm_sec, fmt);

	va_start(list, fmt);
	vfprintf(log_handle, buf, list);
	va_end(list);
}

void Log_Err(const char* fmt, ...)
{
	va_list list;
	time_t _time = time(NULL);
	struct tm* _tm = localtime(&_time);
	char buf[512];
	sprintf(buf, "%u:%02u:%02u [ERROR] %s", _tm->tm_hour, _tm->tm_min, _tm->tm_sec, fmt);

	va_start(list, fmt);
	vfprintf(log_handle, buf, list);
	va_end(list);
}

void PrettyPrint(int color, const char* fmt, ...)
{
	char* _color;
	va_list list;
	char buf[512];
	switch(color)
	{
		case COLOR_GREEN: _color="\e[92m"; break;
		case COLOR_RED: _color= "\e[91m"; break;
		case COLOR_YELLOW: _color= "\e[93m"; break;
		default: _color= "\e[39m";
	}

	sprintf(buf, "%s%s\e[39m", _color, fmt);

	va_start(list, fmt);
	vprintf(buf, list);
	va_end(list);
}

void Log_Fatal(const char* fmt, ...)
{
	va_list list;
	time_t _time = time(NULL);
	struct tm* _tm = localtime(&_time);
	char buf[512];
	sprintf(buf, "\e[91m%u:%02u:%02u [FATAL] %s\e[39m", _tm->tm_hour, _tm->tm_min, _tm->tm_sec, fmt);

	va_start(list, fmt);
	vfprintf(log_handle, buf, list);
	va_end(list);
	exit(1);
}

/* Required to initialize registers to actual memory references */
void init_regs()
{
	reg_bus_coupler_id 			= &modbus_map->tab_input_registers[BUS_COUPLER_ID_REG];
	reg_pdo_size_ao 			= &modbus_map->tab_input_registers[PDO_AO_SIZE_REG];
	reg_pdo_size_ai 			= &modbus_map->tab_input_registers[PDO_AI_SIZE_REG];
	reg_pdo_size_do 			= &modbus_map->tab_input_registers[PDO_DO_SIZE_REG];
	reg_pdo_size_di 			= &modbus_map->tab_input_registers[PDO_DI_SIZE_REG];
	reg_wdt_curr_time 			= &modbus_map->tab_input_registers[WATCHDOG_CURRENT_TIME_REG];
	reg_num_fallbacks_triggered = &modbus_map->tab_input_registers[FALLBACKS_TRIGGERED_REG];
	reg_connections 			= &modbus_map->tab_input_registers[TCP_CONN_NUM_REG];
	reg_hardware_ver 			= &modbus_map->tab_input_registers[HARDWARE_VERSION_REG];
	reg_soft_ver_main 			= &modbus_map->tab_input_registers[SOFTWARE_VERSION_MAIN_REG];
	reg_soft_ver_submain 		= &modbus_map->tab_input_registers[SOFTWARE_VERSION_SUBMAIN_REG];
	reg_soft_ver_beta 			= &modbus_map->tab_input_registers[SOFTWARE_VERSION_BETA_REG];
	reg_serial_num 				= &modbus_map->tab_input_registers[SERIAL_NUM_REG];
	reg_mfg_date_day 			= &modbus_map->tab_input_registers[MFG_DATE_DAY_REG];
	reg_mfg_date_mon 			= &modbus_map->tab_input_registers[MFG_DATE_MONTH_REG];
	reg_mfg_date_year 			= &modbus_map->tab_input_registers[MFG_DATE_YEAR_REG];
	reg_ebus_status 			= &modbus_map->tab_input_registers[0x1040];
	reg_wdt_reset 				= &modbus_map->tab_registers[WATCHDOG_RESET_REG];
	reg_wdt_time 				= &modbus_map->tab_registers[WATCHDOG_TIME_REG];
	reg_wdt_type 				= &modbus_map->tab_registers[WATCHDOG_TYPE_CONF_REG];
	reg_fallback_mode			= &modbus_map->tab_registers[FALLBACK_CONF_REG];
	reg_writelock 				= &modbus_map->tab_registers[WRITELOCK_CONF_REG];
	reg_ebus_ctrl 				= &modbus_map->tab_registers[EBUS_CONF_REG];
	reg_0x1400 					= &modbus_map->tab_registers[0x1400];
	reg_0x1401 					= &modbus_map->tab_registers[0x1401];
	reg_0x1402 					= &modbus_map->tab_registers[0x1402];
	reg_0x1403 					= &modbus_map->tab_registers[0x1403];
	reg_0x1404 					= &modbus_map->tab_registers[0x1404];
	reg_0x1405 					= &modbus_map->tab_registers[0x1405];
	reg_data_start 				= &modbus_map->tab_registers[0x1406];
	reg_term_ids 				= &modbus_map->tab_registers[0x6000];

	/* Initialize to default values */
	*reg_bus_coupler_id = CHARS_TO_INT16('E', 'K');
	*(reg_bus_coupler_id+1) = CHARS_TO_INT16('9', '0');
	*(reg_bus_coupler_id+2) = CHARS_TO_INT16('0', '0');
	*reg_hardware_ver = HARDWARE_VERSION;
	*reg_soft_ver_beta = SOFTWARE_VERSION_BETA;
	*reg_soft_ver_main = SOFTWARE_VERSION_MAIN;
	*reg_soft_ver_submain = SOFTWARE_VERSION_SUBMAIN;
	*reg_serial_num = SERIAL_NUM;
	*reg_mfg_date_day = MFG_DATE_DAY;
	*reg_mfg_date_mon = MFG_DATE_MONTH;
	*reg_mfg_date_year = MFG_DATE_YEAR;
	*reg_wdt_type = WATCHDOG_TYPE_DEFAULT;
	*reg_wdt_time = WATCHDOG_TIME_DEFAULT;
	*reg_fallback_mode = FALLBACK_DEFAULT;
	*reg_writelock = WRITELOCK_ON;
	*reg_num_fallbacks_triggered = 0;
	*reg_wdt_curr_time = *reg_wdt_time;
	*reg_pdo_size_di = 0;
	*reg_pdo_size_do = 0;
	*reg_pdo_size_ai = 0;
	*reg_pdo_size_ao = 0;
	*reg_ebus_ctrl = 1;
	*reg_ebus_status = 1;
}

/*
Every cycle of this loop we need to:
	- Handle new CoE requests
	- Handle old CoE requests that are still waiting on responses
	- Handle watchdog timer updates
	- Handle changes in various things such as:
		- EBUS control register changes
		- Watchdog mode changes
		- Watchdog timer changes
	- Update PDOs from all devices, if EBUS is enabled
	- Check for any other issues
*/
void* ek9000_update(void* vparam)
{
	unsigned write, tgt_term, index, subindex, length, ret;

	/* This is so we add a delay in there */
	static struct
	{
		int valid;
		int write;
		unsigned term;
		union
		{
			coe_read_req_t rreq;
			coe_write_req_t wreq;
		};
	} current_coe_req;
	static int counts = 0;

	/* Check memory regions for any corruption by the slave */
	check_regions();

	/* Process wdt updates */
	if(wdt_triggered && *reg_wdt_type != WATCHDOG_TYPE_DISABLED &&
		fallback_status == 0)
	{
		(*reg_wdt_curr_time) -= 1;
		if(*reg_wdt_curr_time <= 0)
		{
			Log_Warn("[EVENT] Watchdog timer expired.\n");
			ek9000_trigger_fallback();
			*reg_wdt_curr_time = 0;
			fallback_status = 1;
		}
	}

	/* Check if the execute bit is set in 0x1400, indicating we need to execute an operation */
	/* If current_coe_req is VALID, that means we're already doing some CoE IO, so we will ignore this
	   request until the previous request completes */
	if(*reg_0x1400 == 0x1 && !current_coe_req.valid)
	{
		/* write status busy */
		*reg_0x1400 = STATUS_BUSY;
		/* Read the terminal target and the operation */
		write = (*reg_0x1401) & 0x8000;
		tgt_term = (*reg_0x1401) & ~0x8000;
		index = (*reg_0x1402);
		subindex = (*reg_0x1403);
		length = (*reg_0x1404);
		/* Verify that bits 8-15 are NOT used (as per the documentation, although they're ignored) */
		if(subindex & 0xFF00 != 0)
			Log_Warn("[EVENT] Ignored bits in 0x1403 during CoE I/O are set\n");
			/* Check that we arent trying to access an invalid terminal */
		if(tgt_term > nterms || tgt_term == 0)
		{
			Log_Err("[EVENT] Attempt to perform CoE I/O on non-existant terminal. nterms=%u tgt_term=%u\n",
				nterms, tgt_term);
			*reg_0x1400 = STATUS_ERROR;
			
		}
		else if(write)
		{
			/* If we're writing, construct a write request */
			coe_write_req_t req = {
				.length = length,
				.index = index,
				.subindex = subindex,
				.payload = reg_data_start,
			};
			ret = terms[tgt_term]->pfnSendCoEData(contexts[tgt_term], req);
			counts = rand() % 50;
			current_coe_req.wreq = req;
			current_coe_req.write = 1;
			current_coe_req.term = tgt_term;
			*reg_0x1400 = STATUS_BUSY;
		}
		else
		{
			/* Otherwise, construct a read request */
			coe_read_req_t req = {
				.index = index,
				.subindex = subindex,
			};
			counts = rand() % 50;
			current_coe_req.rreq = req;
			current_coe_req.write = 0;
			current_coe_req.term = tgt_term;
			*reg_0x1400 = STATUS_BUSY;
		}
	}
	/* If we've got a coe request currently waiting AND the latency counts are zero,
       we can execute the request! */
	if(current_coe_req.valid && !counts)
	{
		tgt_term = current_coe_req.term;
		if(write)
		{
			ret = terms[tgt_term]->pfnSendCoEData(contexts[tgt_term], 
				current_coe_req.wreq);
			*reg_0x1400 = (ret == 0 ? STATUS_DONE : STATUS_ERROR);
			*reg_0x1405 = ret;
			current_coe_req.valid = 0;
		}
		else
		{
			ret = terms[tgt_term]->pfnReadCoEData(contexts[tgt_term],
				reg_data_start, current_coe_req.rreq);
			*reg_0x1400 = (ret == 0 ? STATUS_DONE : STATUS_ERROR);
			*reg_0x1405 = ret;
			current_coe_req.valid = 0;
		}
	}

	/* Update PDOs from all devices */
	/* NOTE: we do NOT want to update PDOs when the ebus is DOWN */
	for(int i = 0; i < nterms && (*reg_ebus_status); i++)
	{
		if(!terms[i]) continue;
		if(!contexts || !contexts[i]) continue;
		terms[i]->pfnPDOUpdate(contexts[i]);
	}
	
	/* If the data is dirty and needs processing */
	if(dirty)
	{
		
	}

	/* Decrese the fake latency counts */
	counts--;
	
	return NULL;
}

void ek9000_trigger_fallback()
{
	switch (*reg_fallback_mode)
	{
	case FALLBACK_SET_ZERO:
		Log_Warn("[EVENT] Fallback set to zero triggered\n");
		/* Zero ONLY the PDO registers */
		memset(modbus_map->tab_registers, 0, 0x8FF);
		memset(modbus_map->tab_bits, 0, EK9000_COIL_NUM);
		break;
	case FALLBACK_FREEZE:
		Log_Warn("[EVENT] Fallback freeze triggered\n");
		frozen = 1;
		break;
	case FALLBACK_STOP_EBUS:
		Log_Warn("[EVENT] Fallback stop EBUS triggered\n");
		ek9000_stop_ebus();
		break;
	default: Log_Warn("[EVENT] Invalid fallback type in register %x\n", reg_fallback_mode);
		break;
	}
	++*reg_num_fallbacks_triggered;
	fallback_status = 0;
}

void ek9000_reset()
{
	Log_Info("Device reset\n");
	init_regs();
}

void ek9000_stop_ebus()
{
	*reg_ebus_status = 0;
	Log_Info("[EVENT] EBUS stopped\n");
}

void command_exit(int argc, char** argv)
{
	pthread_cancel(listen_thread);
	exit(0);
}

void command_reset_wdt(int argc,char** argv)
{
	*reg_wdt_curr_time = *reg_wdt_time;
	wdt_triggered = 0;
}

void command_reset_defaults(int argc,char** argv)
{

}

void command_help(int argc,char** argv)
{
	for(int i = 0; i < g_ncommands; i++)
		printf("%-20s  - %s\n", g_commands[i].aliases, g_commands[i].helptext);
}

void command_set_wdt_time(int argc, char** argv)
{

}

void command_set_fallback_mode(int argc, char** argv)
{

}

void command_print_settings(int argc, char** argv)
{
	printf("\tWatchdog Current Time:      %u\n", *reg_wdt_curr_time);
	printf("\tWatchdog Time:              %u\n", *reg_wdt_time);
	printf("\tWatchdog Mode:              %u\n", *reg_wdt_type);
	printf("\tFallback mode:              %u\n", *reg_fallback_mode);
	printf("\tWritelock:                  %u\n", *reg_writelock);
	printf("\tEBUS Status:                %u\n", *reg_ebus_status);
	printf("\tEBUS Control:               %u\n", *reg_ebus_ctrl);
	printf("\tNumber of connections:      %u\n", *reg_connections);
	printf("\tFallbacks Triggered:        %u\n", *reg_num_fallbacks_triggered);
	printf("\tAnalog out PDO Size (bits): %u\n", *reg_pdo_size_ao);
	printf("\tAnalog in PDO Size (bits):  %u\n", *reg_pdo_size_ao);
	printf("\tDigi Out PDO Size (bits):   %u\n", *reg_pdo_size_do);
	printf("\tDigi In PDO Size (bits):    %u\n", *reg_pdo_size_di);
}

void command_print_info(int argc, char** argv)
{
	printf("\tHardware version:           %u\n", *reg_hardware_ver);
	printf("\tSoftware version:           %u.%u.%u\n", *reg_soft_ver_main, 
		*reg_soft_ver_submain, *reg_soft_ver_beta);
	printf("\tMfg date:                   %u/%u/%u\n", *reg_mfg_date_day,
		*reg_mfg_date_mon, *reg_mfg_date_year);
}
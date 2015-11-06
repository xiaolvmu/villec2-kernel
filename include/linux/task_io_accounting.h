
struct task_io_accounting {
#ifdef CONFIG_TASK_XACCT
	
	u64 rchar;
	
	u64 wchar;
	
	u64 syscr;
	
	u64 syscw;
#endif 

#ifdef CONFIG_TASK_IO_ACCOUNTING
	u64 read_bytes;

	u64 write_bytes;

	u64 cancelled_write_bytes;
#endif 
};

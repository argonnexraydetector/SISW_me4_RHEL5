--- a/kernel/fork.c	2009-11-30 09:19:34.000000000 +0100
+++ b/kernel/fork.c	2009-11-30 09:19:55.000000000 +0100
@@ -1815,6 +1815,7 @@ void  __mmdrop_delayed(struct mm_struct
 	put_cpu_var(delayed_drop_list);
 }
 
+#ifdef CONFIG_HOTPLUG_CPU
 static void takeover_delayed_drop(int hotcpu)
 {
 	struct list_head *head = &per_cpu(delayed_drop_list, hotcpu);
@@ -1827,6 +1828,7 @@ static void takeover_delayed_drop(int ho
 		__mmdrop_delayed(mm);
 	}
 }
+#endif
 
 static int desched_thread(void * __bind_cpu)
 {

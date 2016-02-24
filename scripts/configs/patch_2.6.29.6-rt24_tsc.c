--- a/arch/x86/kernel/tsc.c	2009-11-27 16:42:20.000000000 +0100
+++ b/arch/x86/kernel/tsc.c	2009-11-27 16:40:13.000000000 +0100
@@ -634,7 +634,7 @@ static int time_cpufreq_notifier(struct
 				void *data)
 {
 	struct cpufreq_freqs *freq = data;
-	unsigned long *lpj, dummy;
+	unsigned long *lpj, dummy = 0;
 
 	if (cpu_has(&cpu_data(freq->cpu), X86_FEATURE_CONSTANT_TSC))
 		return 0;

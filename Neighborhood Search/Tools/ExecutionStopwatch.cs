using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

// https://www.codeproject.com/Articles/31152/ExecutionStopwatch
// https://stackoverflow.com/questions/26472936/why-does-getthreadtimes-return
// Only measures time spent in current thread
namespace DiagnosticsUtils
{
	public class ExecutionStopwatch
	{
		[DllImport("kernel32.dll", SetLastError = true)]
		static extern bool GetThreadTimes(IntPtr hThread,
			out System.Runtime.InteropServices.ComTypes.FILETIME lpCreationTime, out System.Runtime.InteropServices.ComTypes.FILETIME lpExitTime,
			out System.Runtime.InteropServices.ComTypes.FILETIME lpKernelTime, out System.Runtime.InteropServices.ComTypes.FILETIME lpUserTime);

		[DllImport("kernel32.dll")]
		private static extern IntPtr GetCurrentThread();

		[DllImport("kernel32.dll", SetLastError = true)]
		[return: MarshalAs(UnmanagedType.Bool)]
		static extern bool DuplicateHandle(IntPtr hSourceProcessHandle,
			IntPtr hSourceHandle, IntPtr hTargetProcessHandle, out IntPtr lpTargetHandle,
			uint dwDesiredAccess, [MarshalAs(UnmanagedType.Bool)] bool bInheritHandle, uint dwOptions);

		[Flags]
		public enum DuplicateOptions : uint
		{
			DUPLICATE_CLOSE_SOURCE = (0x00000001), // Closes the source handle. This occurs regardless of any error status returned.
			DUPLICATE_SAME_ACCESS = (0x00000002),  // Ignores the dwDesiredAccess parameter. The duplicate handle has the same access as the source handle.
		}

		[DllImport("kernel32.dll")]
		static extern IntPtr GetCurrentProcess();

		private long m_endTimeStamp;
		private long m_startTimeStamp;

		private bool m_isRunning;

		public void Start()
		{
			m_isRunning = true;

			long timestamp = GetThreadTimes();
			m_startTimeStamp = timestamp;
		}

		public void Stop()
		{
			m_isRunning = false;

			long timestamp = GetThreadTimes();
			m_endTimeStamp = timestamp;
		}

		public void Reset()
		{
			m_startTimeStamp = 0;
			m_endTimeStamp = 0;
		}

		public TimeSpan Elapsed
		{
			get
			{
				long elapsed = m_endTimeStamp - m_startTimeStamp;
				TimeSpan result =
					TimeSpan.FromMilliseconds(elapsed / 10000.0);
				return result;
			}
		}

		public bool IsRunning
		{
			get { return m_isRunning; }
		}

		public static long GetThreadTimes()
		{
			//IntPtr threadHandle = GetCurrentThread();
			IntPtr threadHandle;
			IntPtr processHandle = GetCurrentProcess();
			DuplicateHandle(processHandle, GetCurrentThread(), processHandle, 
				out threadHandle, 0, false, (uint)DuplicateOptions.DUPLICATE_SAME_ACCESS);

			/*long notIntersting;
			long kernelTime, userTime;
			long retcode = GetThreadTimes
				(threadHandle, out notIntersting,
				out notIntersting, out kernelTime, out userTime);*/
			bool success = GetThreadTimes(threadHandle, out var _, out var _, 
				out var rawKernelTime, out var rawUserTime);
			ulong uLow = (ulong)rawKernelTime.dwLowDateTime;
			ulong uHigh = (uint)rawKernelTime.dwHighDateTime;
			uHigh = uHigh << 32;
			long kernelTime = (long)(uHigh | uLow);

			uLow = (ulong)rawUserTime.dwLowDateTime;
			uHigh = (uint)rawUserTime.dwHighDateTime;
			uHigh = uHigh << 32;
			long userTime = (long)(uHigh | uLow);

			//bool success = Convert.ToBoolean(retcode);
			/*if (!success)
				throw new Exception(string.Format
				("failed to get timestamp. error code: {0}",
				retcode));*/
			if (!success)
				throw new Exception("Failed to get timestamp");

			long result = kernelTime + userTime;
			return result;
		}
	}
}
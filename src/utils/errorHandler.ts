// Error handling utilities for the hackathon1_book project

export interface ErrorInfo {
  message: string;
  stack?: string;
  component?: string;
  timestamp: Date;
  url?: string;
}

export class AppError extends Error {
  public readonly info: ErrorInfo;

  constructor(message: string, component?: string, originalError?: Error) {
    super(message);
    this.name = 'AppError';

    this.info = {
      message,
      stack: originalError?.stack || this.stack,
      component,
      timestamp: new Date(),
      url: typeof window !== 'undefined' ? window.location.href : undefined
    };
  }
}

export const logError = (error: Error, component?: string): void => {
  const appError = error instanceof AppError ? error : new AppError(error.message, component, error);

  // Log to console in development
  if (process.env.NODE_ENV !== 'production') {
    console.error('Application Error:', appError.info);
  }

  // In a real application, you might send this to an error tracking service
  // For now, we'll just log it
  console.group('Error Details');
  console.log('Message:', appError.info.message);
  console.log('Component:', appError.info.component);
  console.log('Timestamp:', appError.info.timestamp);
  console.log('URL:', appError.info.url);
  console.log('Stack:', appError.info.stack);
  console.groupEnd();
};

export const handleAsyncError = async <T>(
  asyncFn: () => Promise<T>,
  component?: string,
  onError?: (error: AppError) => void
): Promise<T | null> => {
  try {
    return await asyncFn();
  } catch (error) {
    const appError = error instanceof AppError
      ? error
      : new AppError(
          error instanceof Error ? error.message : 'Unknown error occurred',
          component,
          error as Error
        );

    logError(appError, component);

    if (onError) {
      onError(appError);
    }

    return null;
  }
};

export const withErrorHandling = <T extends (...args: any[]) => any>(
  fn: T,
  component?: string,
  onError?: (error: AppError) => void
): T => {
  return function (...args: Parameters<T>): ReturnType<T> {
    try {
      const result = fn.apply(this, args);

      // If it's a promise, handle async errors
      if (result instanceof Promise) {
        return result.catch((error: Error) => {
          const appError = error instanceof AppError
            ? error
            : new AppError(
                error.message,
                component,
                error
              );

          logError(appError, component);

          if (onError) {
            onError(appError);
          }

          // Re-throw to maintain promise rejection behavior
          throw appError;
        }) as ReturnType<T>;
      }

      return result;
    } catch (error) {
      const appError = error instanceof AppError
        ? error
        : new AppError(
            error instanceof Error ? error.message : 'Unknown error occurred',
            component,
            error as Error
          );

      logError(appError, component);

      if (onError) {
        onError(appError);
      }

      throw appError;
    }
  } as T;
};
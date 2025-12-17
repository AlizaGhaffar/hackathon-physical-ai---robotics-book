@echo off
REM Quick deployment script for Hugging Face Spaces
REM Run this from the Desktop directory

echo ========================================
echo RAG Chatbot - Hugging Face Deployment
echo ========================================
echo.

REM Get Space name from user
set /p SPACE_NAME="Enter your Hugging Face Space name (e.g., rag-chatbot-backend): "

if "%SPACE_NAME%"=="" (
    echo Error: Space name cannot be empty!
    pause
    exit /b 1
)

set /p USERNAME="Enter your Hugging Face username: "

if "%USERNAME%"=="" (
    echo Error: Username cannot be empty!
    pause
    exit /b 1
)

echo.
echo Step 1: Cloning Hugging Face Space...
echo.

git clone https://huggingface.co/spaces/%USERNAME%/%SPACE_NAME%

if errorlevel 1 (
    echo.
    echo Error: Failed to clone Space. Please check:
    echo 1. Space exists on Hugging Face
    echo 2. Username and Space name are correct
    echo 3. Git is installed
    pause
    exit /b 1
)

echo.
echo Step 2: Copying backend files...
echo.

REM Copy src folder
xcopy "book\backend\src" "%SPACE_NAME%\src" /E /I /H /Y
if errorlevel 1 goto :copy_error

REM Copy migrations folder
xcopy "book\backend\migrations" "%SPACE_NAME%\migrations" /E /I /H /Y
if errorlevel 1 goto :copy_error

REM Copy individual files
copy "book\backend\alembic.ini" "%SPACE_NAME%\" /Y
copy "book\backend\requirements.txt" "%SPACE_NAME%\" /Y
copy "book\backend\Dockerfile.huggingface" "%SPACE_NAME%\Dockerfile" /Y
copy "book\backend\.env.huggingface.example" "%SPACE_NAME%\.env.example" /Y
copy "book\backend\HUGGINGFACE-DEPLOYMENT.md" "%SPACE_NAME%\README.md" /Y

echo.
echo Step 3: Committing and pushing to Hugging Face...
echo.

cd %SPACE_NAME%

git add .
git commit -m "Deploy RAG chatbot backend to Hugging Face Spaces"
git push

if errorlevel 1 (
    echo.
    echo Error: Failed to push to Hugging Face.
    echo.
    echo You may need to:
    echo 1. Generate a Hugging Face Access Token
    echo 2. Use token as password when Git asks
    echo.
    echo Get token from: https://huggingface.co/settings/tokens
    pause
    exit /b 1
)

echo.
echo ========================================
echo SUCCESS! Backend deployed to Hugging Face
echo ========================================
echo.
echo Your Space URL:
echo https://huggingface.co/spaces/%USERNAME%/%SPACE_NAME%
echo.
echo Next Steps:
echo 1. Go to your Space URL above
echo 2. Click "Settings" tab
echo 3. Add Repository Secrets (see README.md in your Space)
echo 4. Wait for build to complete (check Logs tab)
echo 5. Test: https://%USERNAME%-%SPACE_NAME%.hf.space/api/health
echo.

pause
exit /b 0

:copy_error
echo.
echo Error: Failed to copy files.
echo Please check that you are running this from Desktop directory
echo and that the 'book' folder exists.
pause
exit /b 1
